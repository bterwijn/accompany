#!/usr/bin/python

### This is an auto-generated script - do not edit ###

import subprocess
import time
import sys
sys.path.append('/home/joe/QTProjects/Core/')
import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script
from config import *            # contains configuration details for UH Robot House inc. DB login info

import MySQLdb

#--------------------------------------------------------------------------------
class test(script):

  def Initialize(self):

    if not self.sss.parse:
       rospy.loginfo(" *********** This is the test script ************** ")

# get a DB connection

    if not self.sss.parse:
       try:

          self.conn = MySQLdb.connect(server_config['mysql_log_server'],
                                      server_config['mysql_log_user'],
                                      server_config['mysql_log_password'],
                                      server_config['mysql_log_db'])

       except MySQLdb.Error, e:
          print "Error %d: %s" % (e.args[0], e.args[1])
          sys.exit (1)

       rospy.loginfo('MySQL  initialized')

#--------------------------------------------------------------------------------
  def Run(self):

    overallresult=checkRules(self);        # check the rule set

    if not self.sss.parse:
       rospy.loginfo("SET ::501::TrayIsLowered TO  true")

    if (overallresult):
       if not self.sss.parse:
          cursorSequence = self.conn.cursor()
          sqlSequence = "INSERT INTO SensorLog (timestamp,sensorId,room,channel,value,status) VALUES (NOW(),501\
               ,'','','1','true')"
          cursorSequence.execute(sqlSequence)
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')

    if not self.sss.parse:
       rospy.loginfo("SET ::500::TrayIsRaised TO  false")

    if (overallresult):
       if not self.sss.parse:
          cursorSequence = self.conn.cursor()
          sqlSequence = "INSERT INTO SensorLog (timestamp,sensorId,room,channel,value,status) VALUES (NOW(),500\
               ,'','','0','false')"
          cursorSequence.execute(sqlSequence)
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')


#--------------------------------------------------------------------------------
def checkRules(self):


   # this queries the database rules, executes each and veryfies final result

    overallresult = False    # assume the rule set doesn't apply

   # this extracts the rules set for this script from the database and checks each rule in turn
   # the rules are and'ed or or'ed based on the rule set

    if not self.sss.parse:

         cursorSequence = self.conn.cursor()

         sqlSequence = "SELECT   *  FROM  ActionRules\
                          WHERE     name = 'test'\
                            AND     ruleType = 'R'\
                          ORDER BY  ruleOrder"

         # get the sequence from the sql query

         cursorSequence.execute(sqlSequence)

         sequenceRows = cursorSequence.fetchall()

         # for each row execute the rule to see it is valid

         recCount = 0

         try:
           for sequenceRow in sequenceRows:

              recCount = recCount + 1

              ANDORSwitch = ""

              andOr = sequenceRow[4]

              if andOr== 1:
                 ANDORSwitch = "AND"

              if andOr== 2:
                 ANDORSwitch = "OR"

              rospy.loginfo("%s %s",sequenceRow[5],ANDORSwitch)

              # now do each rule

              cursorRule = self.conn.cursor()

              sqlRule = sequenceRow[6];

              cursorRule.execute(sqlRule)

              ruleRows = cursorRule.fetchone()

              # returning data means that the rule is true

              if (ruleRows==None):
                 rospy.loginfo("%s FALSE",sqlRule)
                 result = False
              else:
                 rospy.loginfo("%s TRUE",sqlRule)
                 result = True 

              if (recCount == 1): 
                 overallresult = result
                 prevANDORSwitch = ANDORSwitch
              else:                                  # now AND or OR each row in turn
                 if (prevANDORSwitch == "OR"):
                    overallresult = overallresult or result
                 if (prevANDORSwitch == "AND"):
                    overallresult = overallresult and result
                 if (prevANDORSwitch == ""):
                    overallresult = overallresult and result;

                 prevANDORSwitch = ANDORSwitch

           if (overallresult):                      # final result
             rospy.loginfo("Total Rule Set is VALID! :) ")
           else:
             rospy.loginfo("Total Rule Set is INVALID :( ")

         except MySQLdb.Error, e:
           rospy.loginfo("Error %d: %s" % (e.args[0],e.args[1]))
           sys.exit(1)  

         finally:   
           cursorSequence.close() 

         return overallresult

#--------------------------------------------------------------------------------
if __name__ == "__main__":
   SCRIPT = test()
   SCRIPT.Start()
#--------------------------------------------------------------------------------
