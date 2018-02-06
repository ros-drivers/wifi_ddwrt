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

import os
import string
import time
import getopt
import re
import csv
import gc
import StringIO
import rospy

from wifi_ddwrt.msg import *
from mechanize import Browser
from std_msgs.msg import Header

import diagnostic_updater
import diagnostic_msgs


class WifiAP:
  """
    WiFiAP Class
  """
  
  def __init__(self, hostname, username, password):
    """
    TODO - description
    :param hostname:
    :param username:
    :param password:
    """
    self.hostname = hostname
    self.username = username
    self.password = password
    
    self.essid = ''
    self.macaddress = ''
    self.channel = ''
    self.rate = ''
    self.tx_power = ''
    
    self.ap_ok = False
    self.last_ex = ' '
    
    # List of interfaces with the following structure
    # client = {'macaddr': None, 'signal': None, 'noise': None, 'snr': None, 'quality': None, 'nt_devices': []}
    self.interfaces = {}
  
  # ----------------------------------------------------------------------------------------------------------------- #
  
  def newBrowser(self):
    """
    Creates a Browser object responsible for access the DDWRT page and gather the necessary information
    :return:
    """
    # Create new browsers all the time because its data structures grow
    # unboundedly (texas#135)
    br = Browser()
    br.add_password(self.hostname, self.username, self.password)
    br.set_handle_robots(None)
    return br
  
  # ----------------------------------------------------------------------------------------------------------------- #
  
  def fetchSiteSurvey(self):
    """
    Fetch the survey info from the DDWRT router
    :return: A list of networks found by the survey (Network.msg)
    """
    try:
      url = 'http://{0}/Site_Survey.asp'.format(self.hostname)
      response = self.newBrowser().open(url)
      body = response.read()
      # make sure that we put a stamp on things
      header = Header()
      header.stamp = rospy.Time.now()
      networks = []
      survey = SiteSurvey(header, networks)
      lines = body.split('\n')
      for i in range(len(lines)):
        if lines[i].startswith('var table = '):
          break
      aplines = []
      for j in range(i + 1, len(lines)):
        if lines[j].startswith(');'):
          break
        line = lines[j].strip()
        if not line:
          continue
        if line[0] == ',':
          line = line[1:]
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
    except Exception as ex:
      rospy.logwarn('Error fetching data from the Site Survey, with the following message: {0}'.format(ex))
    return survey
  
  # ----------------------------------------------------------------------------------------------------------------- #
  
  def fetchBandwidthStats(self, interface):
    """
    TODO - add description
    :param interface:
    :return:
    """
    parts = []
    try:
      url = 'http://{0}/fetchif.cgi?{1}'.format(self.hostname, interface)
      response = self.newBrowser().open(url)
      body = response.read()
      lines = body.split('\n')
      if len(lines) > 1:
        line = lines[1].strip()
        iparts = line.split(':', 1)
        parts = iparts[1].split()
    except Exception as ex:
      last_ex = 'Error fetching data from the interface {0} with the error {1}'.format(interface, ex)
      rospy.logwarn(last_ex)
      self.ap_ok = False
    return parts
  
  # ----------------------------------------------------------------------------------------------------------------- #
  
  def fetchCurrentAP(self):
    """
    TODO - add description
    :return:
    """
    try:
      url = 'http://{0}/Status_Wireless.live.asp'.format(self.hostname)
      response = self.newBrowser().open(url)
      body = response.read()
      lines = re.findall(r'\{(.*?)\}', body)
      d = {}  # dictionary
      line = None
      for line in lines:
        parts = line.split("::", 1)
        if len(parts) == 2:
          d[parts[0]] = parts[1]
      
      self.essid = d.get('wl_ssid', '')
      self.macaddress = d.get('wl_mac', '')
      self.channel = int(d.get('wl_channel', '').split()[0])
      self.rate = d.get('wl_rate', '')
      self.tx_power = d.get('wl_xmit', '')
      
      active_wireless = d.get('active_wireless', None)
      
      self.interfaces = {}
      if active_wireless:
        active_wireless = active_wireless.replace("'", "")  # we want to remove the apostrophe
        l_devices = active_wireless.split(',')
        
        if (len(l_devices) % 10) == 0:
          clients = zip(*[iter(l_devices)] * 10)
        elif (len(l_devices) % 9) == 0:
          clients = zip(*[iter(l_devices)] * 9)
        elif (len(l_devices) % 8) == 0:
          clients = zip(*[iter(l_devices)] * 8)
        else:
          raise Exception("Unable to unpack AP clients. Router maybe incompatible, please tell the maintainer.")
          
        # clean all the information about the interfaces
        self.interfaces = {}
        # grab all the client interfaces an their status
        for client in clients:
          macaddr = client[0]
          interface = client[1]
          signal = 9999
          noise = 9999
          snr = 9999
          quality = 9999
          
          try:
            # netgear R7800
            if len(client) == 10:
              signal = int(client[6])
              noise = int(client[7])
              snr = int(client[8])
              quality = int(client[9]) / 10
            
            # old devices (Not sure if this is really needed but maintained for compatibility)
            elif len(client) == 7:
              signal = int(client[4])
              noise = int(client[5])
              snr = int(client[6])
              quality = signal * 1.24 + 116
            
            elif len(client) == 9:
              signal = int(client[5])
              noise = int(client[6])
              snr = int(client[7])
              quality = int(client[8]) / 10
            
            else:
              raise Exception("Unable to unpack AP clients. Router maybe incompatible, please tell the maintainer.")
          
          except Exception as ex:
            raise Exception('Unable to read data from interface {0} with error: {1}'.format(interface, ex))
            rospy.logwarn(last_ex)
          
          nt_devices = self.fetchBandwidthStats(interface)
          self.interfaces.setdefault(interface, []).append({'macaddr': macaddr,
                                                            'signal': signal,
                                                            'noise': noise,
                                                            'snr': snr,
                                                            'quality': quality,
                                                            'nt_devices': nt_devices})
        self.last_ex = ''
        self.ap_ok = True
    
    except Exception as ex:
      last_ex = 'Unable to access the AP at {0}. Error: {1}'.format(self.hostname, ex)
      rospy.logwarn(last_ex)
      self.ap_ok = False
  
  # ------------------------------------------------------------------------------------------------------------------- #
  
  def produce_diagnostics(self, stat):
    
    """
    This creates a diagnostic message an sent it to the Diagnostic Aggregator
    :param stat:
    :return:
    """
    stat.clearSummary()
    try:
      if self.ap_ok:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'AP {0} [{1}] working OK'.format(self.essid, self.hostname))
      else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, self.last_ex)
      
      stat.add('essid', self.essid)
      stat.add('channel', self.channel)
      stat.add('rate', self.rate)
      stat.add('tx_power', self.tx_power)
      
      for it in self.interfaces.keys():
        stat.add(it, self.interfaces[it])
    
    except Exception as ex:
      error = 'Unable to access the AP at {0}. Error: {1}'.format(self.hostname, ex)
      rospy.logwarn(error)
      stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, error)
    finally:
      return stat
  
  # ------------------------------------------------------------------------------------------------------------------ #


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #


def loop():
  """
  TODO - add description
  :return:
  """
  rospy.init_node("wifi_ddwrt")
  
  router_ip = rospy.get_param('~hostname', '192.168.1.1')
  username = rospy.get_param('~username', 'admin')
  password = rospy.get_param('~password', 'admin')
  node_rate = rospy.get_param('~node_rate', 0.5)
  diag_rate = rospy.get_param('~diag_rate', 1)
  diag_agg_topic = rospy.get_param('~diag_agg_topic', 'wifi_ap')
  site_survey_update = rospy.get_param('~site_survey_update', 120)
  site_survey_topic = rospy.get_param('~site_survey_topic', 'ddwrt/sitesurvey')
  hw_id = rospy.get_param('~hd_id', 'RouterXPTO')
  
  
  pub1 = rospy.Publisher(site_survey_topic, SiteSurvey, queue_size=1)
  
  ap = WifiAP(router_ip, username, password)
  
  updater = diagnostic_updater.Updater()
  updater.add(diag_agg_topic, ap.produce_diagnostics)
  updater.setHardwareID(hw_id)
  updater.period = diag_rate
  
  r = rospy.Rate(node_rate)
  last_time = 0
  
  while not rospy.is_shutdown():
    # Needed because mechanize leaves data structures that the GC sees as uncollectable (texas#135)
    breakUpTrash()
    
    if time.time() - last_time > site_survey_update:
      try:
        survey = ap.fetchSiteSurvey()
        pub1.publish(survey)
        last_time = time.time()
      except Exception as e:
        rospy.logwarn("Caught exception on [fetchSiteSurvey]: %s" % e)
    
    ap.fetchCurrentAP()
    updater.update()
    
    r.sleep()


# ------------------------------------------------------------------------------------------------------------------- #

def breakUpTrash():
  """
  TODO - add description
  :return:
  """
  
  for item in gc.garbage:
    if type(item) == dict:
      for s in item.keys_iter():
        del item[s]
    del gc.garbage[:]


# ------------------------------------------------------------------------------------------------------------------- #

def test():
  """
  TODO - add description
  :return:
  """
  router_ip = rospy.get_param('~hostname', '192.168.1.1')
  username = rospy.get_param('~username', 'admin')
  password = rospy.get_param('~password', 'admin')
  
  ap = WifiAP(router_ip, username, password)
  while 1:
    if 0:
      survey = ap.fetchSiteSurvey()
      print
      survey
    if 1:
      node = ap.fetchCurrentAP()
      print
      node


# ------------------------------------------------------------------------------------------------------------------- #

def usage(progname):
  """
  TODO - add description
  :param progname:
  :return:
  """
  print
  __doc__ % vars()


# ------------------------------------------------------------------------------------------------------------------- #

def main(argv, stdout, environ):
  """
  Main execution
  :param argv:
  :param stdout:
  :param environ:
  :return:
  """
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", ])
  
  testflag = 0
  
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--test":
      testflag = 1
  
  if testflag:
    test()
    return
  
  loop()


# ------------------------------------------------------------------------------------------------------------------- #

if __name__ == "__main__":
  """
  Main
  """
  main(sys.argv, sys.stdout, os.environ)

# EOF
