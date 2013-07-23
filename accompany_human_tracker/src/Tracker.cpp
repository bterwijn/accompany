#include <Tracker.h>

using namespace std;

/**
 * Constructor
 * @param trackedHumansPub 'trackedHuman' publisher
 * @param markerArrayPub 'markerArray' publisher for visualization purposes
 * @param priorHull the area where people are detected
 * @param entryExitHulls the entry and exit areas of the scene
 * @param stateThreshold matching threshold on distance
 * @param appearanceThreshold matching threshold on appearance
 * @param appearanceUpdate weight of new appearance vs old one
 * @param maxSpeed the maximum speed of a track
 * @param minMatchCount the minimum number of matches before a track is published
 * @param maxUnmatchCount the maximum number of consecutive non-matches before a track might be removed 
 */
Tracker::Tracker(const ros::Publisher& trackedHumansPub,
                 const ros::Publisher& markerArrayPub,
                 const std::vector<WorldPoint>& priorHull,
                 const std::vector< std::vector<WorldPoint> >& entryExitHulls,
                 double stateThreshold,
                 double appearanceThreshold,
                 double appearanceUpdate,
                 double maxSpeed,
                 double maxCovar,
                 unsigned minMatchCount,
                 unsigned maxUnmatchCount)
{
  this->trackedHumansPub=trackedHumansPub;
  this->markerArrayPub=markerArrayPub;
  this->priorHull=priorHull;
  this->entryExitHulls=entryExitHulls;
  this->stateThreshold=stateThreshold;
  this->appearanceThreshold=appearanceThreshold;
  this->appearanceUpdate=appearanceUpdate;
  this->maxSpeed=maxSpeed;
  this->maxCovar=maxCovar;
  this->minMatchCount=minMatchCount;
  this->maxUnmatchCount=maxUnmatchCount;
  // kalman motion and observation model
  const double tm[]={1,0,1,0,
                     0,1,0,1,
                     0,0,1,0,
                     0,0,0,1};
  transModel=vnl_matrix<double>(tm,4,4);
  const double tc[]={5,0,0,0,
                     0,5,0,0,
                     0,0,5,0,
                     0,0,0,5};
  transCovariance=vnl_matrix<double>(tc,4,4);
  const double om[]={1,0,0,0,
                     0,1,0,0};
  obsModel=vnl_matrix<double>(om,2,4);
  const double oc[]={10,0,
                     0,10};
  obsCovariance=vnl_matrix<double>(oc,2,2);
  coordFrame="";// set coordinate frame of HumanDetections to unkown
  trackerCount=0;
}


WorldPoint toWorldPoint(const accompany_uva_msg::HumanDetection& detection)
{
  WorldPoint wp(detection.location.point.x,
                detection.location.point.y,
                detection.location.point.z);
  wp*=1000.0; // from meters to millimeters
  return wp;
}

double timeDiff(const struct timeval& time,
                const struct timeval& prevTime)
{
  return (time.tv_sec-prevTime.tv_sec)+(time.tv_usec-prevTime.tv_usec)/((double)1E6);
}

bool randomTrackCreation()
{
  //double r=(rand()/(double)RAND_MAX);
  //return (r<0.03);
  return false;
}

/**
 * Processes human detections, assign detections to known tracks or create new tracks
 * @param humanDetections the detections
 */
void Tracker::processDetections(const accompany_uva_msg::HumanDetections::ConstPtr& humanDetections)
{
  cout<<"-received "<<humanDetections->detections.size()<<" HumanDetections"<<endl;
  //for (unsigned i=0;i<humanDetections->detections.size();i++)
  //  cout<<humanDetections->detections[i]<<endl;
  if (humanDetections->detections.size()>0)
    coordFrame=humanDetections->detections[0].location.header.frame_id;

  // transition based on elapsed time
  struct timeval time;
  gettimeofday(&time, NULL);
  if (tracks.size()>0)
  {
    double diff=timeDiff(time,prevTime);
    transModel[0][2]=diff; // set time past so the speed vector get multiplied before adding to the position vector
    transModel[1][3]=diff;
  }
  for (unsigned i=0;i<tracks.size();i++)
    tracks[i].transition(transModel,transCovariance,maxCovar);

  // associate observations with tracks
  dataAssociation.clear(tracks.size(),humanDetections->detections.size());
  for (unsigned i=0;i<tracks.size();i++)
    for (unsigned j=0;j<humanDetections->detections.size();j++)
    {
      double match=tracks[i].match(humanDetections->detections[j],
                                   obsModel,
                                   stateThreshold,
                                   appearanceThreshold);
      dataAssociation.set(i,j,match);
    }

  vector<int> associations=dataAssociation.associate(stateThreshold+appearanceThreshold);

  // print associations
  /*
  cout<<"associations:"<<endl;
  for (unsigned i=0;i<associations.size();i++)
    cout<<associations[i]<<" ";
  cout<<endl;
  */

  // add unmatch count of tracks
  for (unsigned i=0;i<tracks.size();i++)
    tracks[i].addUnmatchCount();

  // update or create tracks
  int assignedCount=0;
  for (unsigned i=0;i<associations.size();i++)
  {
    if (associations[i]<0) // not assigned
    {
      if (inside(toWorldPoint(humanDetections->detections[i]),entryExitHulls) ||
          randomTrackCreation())
      {
        cout<<"unassociated detection in entryExitHulls, new track started"<<endl;
        tracks.push_back(Track(humanDetections->detections[i],maxCovar));
      }
      else
        cout<<"unassociated detection NOT in entryExitHulls, ignore"<<endl;
    }
    else // assigned
    {
      assignedCount++;
      vnl_vector<double> speedBefore=tracks[associations[i]].getSpeed();
      tracks[associations[i]].observation(humanDetections->detections[i],
                                          appearanceUpdate,
                                          obsModel,
                                          obsCovariance,
                                          maxCovar);
      vnl_vector<double> speedAfter=tracks[associations[i]].getSpeed();
      tracks[associations[i]].maxSpeed(maxSpeed);
      tracks[associations[i]].humanProb+=detect_human_score; // more likely a human
      double sp=speedAfter.two_norm();
      if (sp>0.1)
      {
        tracks[associations[i]].humanProb+=velocity_human_score; // even more likely a human
        double smoothSp=(speedAfter-speedBefore).two_norm();
        if (smoothSp<sp*0.1)
        {
          tracks[associations[i]].humanProb+=smooth_velocity_human_score; // even more likely a human
        }
      }
    }
  }
  normalizeHumanProb();
  cout<<" trackerCount: "<<trackerCount<<" assignedCount: "<<assignedCount<<endl;
  
  removeTracks();
  reduceSpeed();

  cout<<*this<<endl;
  prevTime=time;
  
  publishTracks();
  trackerCount++;
}

/**
 * Processes people face dections. Assign names of people to closest track.
 * @param detectionArray people face dections
 */
void Tracker::identityReceived(const cob_people_detection_msgs::DetectionArray::ConstPtr& detectionArray)
{
  cout<<"-received "<<detectionArray->detections.size()<<" face detections"<<endl;
  if (coordFrame.size()>0) // if HumanDetections coordinate frame is known
  {
    for (unsigned i=0;i<detectionArray->detections.size();i++)
    {
      geometry_msgs::PoseStamped pose=detectionArray->detections[i].pose;
      try// transform to HumanDetections coordinate system
      {
        geometry_msgs::PoseStamped transPose;
        /*transformListener.waitForTransform(pose.header.frame_id,coordFrame,pose.header.stamp,ros::Duration(.3));
        transformListener.transformPose(coordFrame,
                                        pose,
                                        transPose); */ // to slow
        ros::Time past=ros::Time::now()-ros::Duration(0.1); // time slightly in past to trick tf into tranforming imediately
        transformListener.transformPose(coordFrame,
                                        past,
                                        pose,
                                        pose.header.frame_id,
                                        transPose);
        geometry_msgs::PointStamped transTFPoint;
        transTFPoint.header=transPose.header;
        transTFPoint.point=transPose.pose.position;
        label(transTFPoint,detectionArray->detections[i].label);
      }
      catch (tf::TransformException e)
      {
        cerr<<"error while tranforming human location: "<<e.what()<<endl;
      }
    }
  }
}

/**
 * Processes robot position. Assign name 'robot' to closest track.
 * @param tf tf coordinate frame
 */
void Tracker::tfCallBack(const tf::tfMessage& tf)
{  
  for (unsigned i=0;i<tf.transforms.size();i++)
  {
    if (tf.transforms[i].child_frame_id=="/base_link") // if robot
    {
      cout<<"-received robot position"<<endl;
      
      if (coordFrame.size()>0) // if HumanDetections coordinate frame is known
      {
        try// transform to HumanDetections coordinate system
        {
          geometry_msgs::PointStamped tfPoint;
          tfPoint.header=tf.transforms[i].header;
          tfPoint.point.x=tf.transforms[i].transform.translation.x;
          tfPoint.point.y=tf.transforms[i].transform.translation.y;
          tfPoint.point.z=tf.transforms[i].transform.translation.z;
          geometry_msgs::PointStamped transTFPoint;
          //transformListener.waitForTransform(tfPoint.header.frame_id,coordFrame,tfPoint.header.stamp,ros::Duration(.01));
          //transformListener.transformPoint(coordFrame,
                                           //tfPoint,
                                           //transTFPoint); // to slow
          ros::Time past=ros::Time::now()-ros::Duration(0.1); // time slightly in past to trick tf into tranforming imediately
          transformListener.transformPoint(coordFrame,
                                           past,
                                           tfPoint,
                                           tfPoint.header.frame_id,
                                           transTFPoint);
	  //label(transTFPoint,"robot");
          int robotIndex=getRobotIndex();
          if (robotIndex>=0) // have robot Track
          {
            tracks[robotIndex].updateRobot(transTFPoint,
                                           obsModel,
                                           obsCovariance,
                                           maxCovar);
          }
          else // no robot Track
          {
            cout<<"unassociated detection in entryExitHulls, new track started"<<endl;
            tracks.push_back(Track(transTFPoint,maxCovar));
          }
        }
        catch (tf::TransformException e)
        {
          cerr<<"error while tranforming robot location: "<<e.what()<<endl;
        }
      }
      break;
    }
  }
}

/**
 * Label the track that is closest to point with label.
 * @param point position of object
 * @param label name of object
 */
void Tracker::label(geometry_msgs::PointStamped point,string label)
{
  WorldPoint wp(point.point.x,
                point.point.y,
                point.point.z);
  wp*=1000.0; // from meters to millimeters
  double maxDistance=numeric_limits<double>::max();
  int best=-1;
  for (unsigned i=0;i<tracks.size();i++)
  {
    if (tracks[i].matchCount>=minMatchCount) // only tracks with proper match count
    {
      double distance=wp.squareDistance(tracks[i].toWorldPoint());
      if (distance<1 && distance<maxDistance)
      {
        maxDistance=distance;
        best=(int)(i);
      }
    }
  }
  if (best>=0)
  {
    idToName.setIDName(tracks[best].getID(),label);
    cout<<"label track "<<tracks[best].getID()<<" '"<<label<<"'"<<endl;
  }
}

/**
 * Reduce speed when unmatched so not to move far away from last matching observation
 */
void Tracker::reduceSpeed()
{
  for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();it++)
  {
    if (it->unmatchedCount>0) // if not matched in last cycle
      it->reduceSpeed();
  }
}

/**
 * Remove tracks when unmatched for more than maxUnmatchCount and (in entryExitHulls or not in priorHull);
 */
void Tracker::removeTracks()
{
  for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();)
  {
    bool remove=false;
    if (it->unmatchedCount>=maxUnmatchCount) // if not matched for some time
    {
      if (inside(it->toWorldPoint(),entryExitHulls)) // if in entryExitHulls
      {
        cout<<"remove track because unmatched="<<it->unmatchedCount<<" and is in entryExitHulls"<<endl;
        tracks.erase(it);
        remove=true;
      }
      /*
      if (!inside(it->toWorldPoint(),priorHull)) // if not in priorHull
      {
        cout<<"remove track because unmatched="<<it->unmatchedCount<<" and is out of priorHull"<<endl;
        tracks.erase(it);
        remove=true;
      }
      */
    }
    /*
    if (it->unmatchedCount>it->matchCount)
    {
      cout<<"remove track because unmatched="<<it->unmatchedCount<<" > matchCount"<<it->matchCount<<endl;
      tracks.erase(it);
      remove=true;
    }
    */
    if (!remove) it++;
  }
}

/**
 * Publish the known tracks.
 */
void Tracker::publishTracks()
{
  accompany_uva_msg::TrackedHumans trackedHumans;
  accompany_uva_msg::TrackedHuman trackedHuman;
  trackedHuman.specialFlag=0;
  trackedHuman.location.header.stamp=ros::Time::now();
  trackedHuman.location.header.frame_id=coordFrame;
  int humanIndex=getMaxHumanProbIndex();
  for (unsigned i=0;i<tracks.size();i++)
  {
    if (tracks[i].matchCount>=minMatchCount) // only tracks with proper match count
    {
      tracks[i].writeMessage(trackedHuman);
      string name=idToName.getIDName(tracks[i].getID());

      trackedHuman.specialFlag=0;
      if (tracks[i].isRobot())
      {
        trackedHuman.identity="robot";
      }
      else
      {
        if (name.compare(IDToName::unkown)!=0)
          trackedHuman.identity=name;
        else
          trackedHuman.identity="";
        if (i==humanIndex)
          trackedHuman.specialFlag=1;
      }
      trackedHumans.trackedHumans.push_back(trackedHuman);
    }
  }
  trackedHumansPub.publish(trackedHumans);
  markerArrayPub.publish(msgToMarkerArray.toMarkerArray(trackedHumans,"trackedHumans")); // publish visualisation
}

/**
 * ostream a tracker
 */
std::ostream& operator<<(std::ostream& out,const Tracker& tracker)
{
  out<<"Tracker ("<<tracker.tracks.size()<<"):"<<endl;
  out<<tracker.dataAssociation;
  for (unsigned i=0;i<tracker.tracks.size();i++)
    out<<"["<<i<<"] "<<tracker.tracks[i];
  out<<endl;
  return out;
}

void Tracker::normalizeHumanProb()
{
  double sum=0;
  for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();it++)
    sum+=it->humanProb;
  if (sum>0)
    for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();it++)
      it->humanProb/=sum;
}

int Tracker::getMaxHumanProbIndex()
{
  int index=-1;
  double max=0;
  for (unsigned i=0;i<tracks.size();i++)
  {
    if (!tracks[i].isRobot())
    {
      if (tracks[i].humanProb>max)
      {
        max=tracks[i].humanProb;
        index=i;
      }
    }
  }
  return index;
}

int Tracker::getRobotIndex()
{
  for (unsigned i=0;i<tracks.size();i++)
    if (tracks[i].isRobot())
      return i;
  return -1;
}
