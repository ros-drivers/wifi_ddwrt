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

## chat is a simple IM-like test node. It demonstrates publishing and
## subscribing to the same topic.

PKG = 'wifi_ddwrt'
import roslib; roslib.load_manifest(PKG) 

import os, sys, string, time, getopt, re
import StringIO

import rospy
from wifi_ddwrt.msg import *
from pr2_msgs.msg import AccessPoint

from mechanize import Browser
from roslib.msg import Header
import csv

class WifiAP:
  def __init__(self, hostname, username, password):
    self.hostname = hostname
    self.br = Browser()
    self.br.add_password(hostname, username, password)
    self.br.set_handle_robots(None)

  def fetchSiteSurvey(self):
    url = "http://%s/Site_Survey.asp" % self.hostname

    response = self.br.open(url)

    body = response.read()
    
    #make sure that we put a stamp on things
    header = Header()
    header.stamp = rospy.Time.now()
    networks = []
    survey = SiteSurvey(header, networks)

    lines = body.split("\n")
    for i in range(len(lines)):
      if lines[i].startswith("var table = "):
        break

    aplines = []
    for j in range(i+1, len(lines)):
      if lines[j].startswith(");"): break
      line = lines[j].strip()
      if not line: continue
      if line[0] == ",": line = line[1:]

      aplines.append(line)

    fp = StringIO.StringIO(string.join(aplines, '\n'))
    reader = csv.reader(fp)
    for row in reader:
      essid = row[0]
      macattr = row[2]
      channel = int(row[3])
      rssi = int(row[4])
      noise = int(row[5])
      beacon = int(row[6])

      network = Network(macattr, essid, channel, rssi, noise, beacon)
      survey.networks.append(network)
    return survey

  def fetchBandwidthStats(self, interface):
    url = "http://%s/fetchif.cgi?%s" % (self.hostname, interface)
    response = self.br.open(url)
    body = response.read()

    lines = body.split("\n")

    if len(lines) > 1:
      line = lines[1].strip()
      iparts = line.split(":", 1)
      parts = iparts[1].split()
      print interface, parts
      

  def fetchCurrentAP(self):
    url = "http://%s/Status_Wireless.live.asp" % self.hostname
    response = self.br.open(url)
    body = response.read()

    line = None
    lines = body.split("\n")

    d = {}
    for line in lines:
      line = line[1:-1]
      line = line.replace("&nbsp;", "")
      parts = line.split("::", 1)
      if len(parts) == 2:
        d[parts[0]] = parts[1]
      
    essid = d.get('wl_ssid', '')
    wl_channel = d.get('wl_channel', '').split()[0]
    channel = int(wl_channel)
    rate = d.get('wl_rate', '')
    
    signal = None
    noise = None
    snr = None
    quality = None

    tx_power = d.get('wl_xmit', '')

    active_wireless = d.get('active_wireless', None)
    if active_wireless:
      active_wireless = active_wireless.replace("'", "")
      parts = active_wireless.split(",")
      macaddr = parts[0]
      interface = parts[1]
      if len(parts) == 7:
        signal = int(parts[4])
        noise = int(parts[5])
        snr = int(parts[6])
        quality = signal * 1.24 + 116
      else:
        signal = int(parts[5])
        noise = int(parts[6])
        snr = int(parts[7])
        quality = int(parts[8])/10
      
      #self.fetchBandwidthStats(interface)

    #make sure that we put a stamp on things
    header = Header()
    header.stamp = rospy.Time.now()

    ap = AccessPoint(header=header,
                     essid=essid,
                     macaddr=macaddr,
                     signal=signal,
                     noise=noise,
                     snr=snr,
                     channel=channel,
                     rate=rate,
                     quality=quality,
                     tx_power=tx_power)
    return ap


def loop():
  rospy.init_node("wifi_ddwrt")

  router_ip = rospy.get_param('~router', 'wifi-router')
  username = rospy.get_param('~username', 'root')
  password = rospy.get_param('~password', '')

  ap = WifiAP(router_ip, username, password)

  pub1 = rospy.Publisher("ddwrt/sitesurvey", SiteSurvey)
  pub2 = rospy.Publisher("ddwrt/accesspoint", AccessPoint)

  r = rospy.Rate(.5)
  lastTime = 0
  while not rospy.is_shutdown():
    if time.time() - lastTime > 60:
      survey = ap.fetchSiteSurvey()
      pub1.publish(survey)
      lastTime = time.time()
    node = ap.fetchCurrentAP()
    if node: pub2.publish(node)
    r.sleep()
        
def test():
  router_ip = rospy.get_param('~router_ip', 'wifi-router')
  username = rospy.get_param('~username', 'root')
  password = rospy.get_param('~password', '')

  ap = WifiAP(router_ip, username, password)
  while 1:
    if 0:
      survey = ap.fetchSiteSurvey()
      print survey
    if 1:
      node = ap.fetchCurrentAP()
      print node

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1

  if testflag:
    test()
    return

  loop()

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
        

