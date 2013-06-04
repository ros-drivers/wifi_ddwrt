#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: gossipbot.py 1013 2008-05-21 01:08:56Z sfkwc $

import os, sys, string, time, getopt, re
import StringIO
import random

import rospy
from wifi_ddwrt.msg import *
from pr2_msgs.msg import AccessPoint

from std_msgs.msg import Header

wifi_data = {"header": {"seq": 24, "stamp": 1257901832.6, "frame_id": "0"}, "networks": [{"macaddr": "00:18:F8:F9:6C:41", "essid": "willow", "channel": 11, "rssi": -35, "noise": -88, "beacon": 100}, {"macaddr": "00:18:F8:F9:6B:BD", "essid": "willow", "channel": 11, "rssi": -71, "noise": -88, "beacon": 100}, {"macaddr": "00:18:F8:F9:6C:1D", "essid": "willow", "channel": 1, "rssi": -70, "noise": -92, "beacon": 100}, {"macaddr": "00:30:44:03:1F:F9", "essid": "PRLAN", "channel": 1, "rssi": -73, "noise": -92, "beacon": 100}, {"macaddr": "00:30:44:03:1F:F4", "essid": "PRGLAN", "channel": 2, "rssi": -80, "noise": -89, "beacon": 100}, {"macaddr": "00:18:F8:F9:6C:4D", "essid": "willow", "channel": 6, "rssi": -81, "noise": -86, "beacon": 100}, {"macaddr": "00:18:F8:F9:6C:44", "essid": "willow", "channel": 6, "rssi": -79, "noise": -85, "beacon": 100}]}

def minmax(v, lower, upper):
    if v < lower: v = lower
    if v > upper: v = upper
    return v

class WifiAP:
  def __init__(self):
      self.ap = AccessPoint()
      self.t = time.time()
      self._pick_ap()
      
  def _pick_ap(self):
      ap = random.choice(wifi_data['networks'])
      self.ap.macaddr = ap['macaddr']
      self.ap.channel = ap['channel']
      self.ap.essid = ap['essid']
      self.ap.rate = '54 Mbps'
      self.ap.tx_power = '71 mW'
      self.ap.signal = random.randint(-100, -48)
      self.ap.snr = 44
      self.ap.noise = -92
      self.ap.quality = 56

  def _gen_snr(self):
      d = 2
      if time.time() - self.t > 20:
          d = 10
      self.ap.signal = minmax(self.ap.signal + random.randint(-d, d), -70, -20)

      self.ap.snr = minmax(self.ap.snr + random.randint(-d, d), 30, 90)
      self.ap.noise = minmax(self.ap.noise + random.randint(-d, d), -95, -30)
      self.ap.quality = int(self.ap.signal * 1.24 + 116)
  
  def fetchSiteSurvey(self):
    header = Header()
    header.stamp = rospy.Time.now()
    networks = []
    survey = SiteSurvey(header, networks)
    
    for network in wifi_data["networks"]:
      network = Network(network['macaddr'], network['essid'], network['channel'], network['rssi'], network['noise'], network['beacon'])
      survey.networks.append(network)
    return survey

  def fetchCurrentAP(self):
    if time.time() - self.t > 60:
        self._pick_ap()
    self._gen_snr()
    
    #make sure that we put a stamp on things
    self.ap.header = Header()
    self.ap.header.stamp = rospy.Time.now()

    return self.ap

def loop():
  rospy.init_node("wifi_ddwrt")

  ap = WifiAP()

  pub1 = rospy.Publisher("ddwrt/sitesurvey", SiteSurvey)
  pub2 = rospy.Publisher("ddwrt/accesspoint", AccessPoint)

  while not rospy.is_shutdown():
    survey = ap.fetchSiteSurvey()
    pub1.publish(survey)
    node = ap.fetchCurrentAP()
    pub2.publish(node)
    time.sleep(5)
        
def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help",])

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return

  loop()

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
        

