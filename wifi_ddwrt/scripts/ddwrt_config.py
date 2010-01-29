
import os, sys
import urllib2, urllib
import base64
import StringIO
import string
import csv

class DDWRT:
    def __init__(self, hostname, username, password):
        self.hostname = hostname
        self.username = username
        self.password = password

    def fetch(self, url, args=None):
        req = urllib2.Request(url)
        base64string = base64.encodestring("%s:%s" % (self.username, self.password))[:-1]

        req.add_header("Authorization", "Basic %s" % base64string)
        if args:
            req.data = urllib.urlencode(args)

        fp = urllib2.urlopen(req)
        body = fp.read()
        fp.close()

        return body

    def connect(self, essid):
        url = "http://%s/apply.cgi" % self.hostname

        args = {}
        args["submit_button"]="Wireless_Basic"
        args["action"]="ApplyTake"
        args["change_action"]="gozila_cgi"
        args["submit_type"]="save"
        args["wl0_nctrlsb"]=""
        args["wl1_nctrlsb"]=""
        args["iface"]=""
        args["wl0_mode"]="sta"
        args["wl0_net_mode"]="n-only"
        args["wl0_ssid"]=essid
        args["wl0_distance"]="500"
        args["eth1_bridged"]="1"
        args["eth1_multicast"]="0"
        args["eth1_nat"]="1"
        args["eth1_ipaddr"]="4"
        args["eth1_ipaddr_0"]="0"
        args["eth1_ipaddr_1"]="0"
        args["eth1_ipaddr_2"]="0"
        args["eth1_ipaddr_3"]="0"
        args["eth1_netmask"]="4"
        args["eth1_netmask_0"]="0"
        args["eth1_netmask_1"]="0"
        args["eth1_netmask_2"]="0"
        args["eth1_netmask_3"]="0"
        args["wl1_mode"]="ap"
        args["wl1_net_mode"]="disabled"
        args["wl1_ssid"]="dd-wrt"
        args["wl1_closed"]="0"
        args["wl1_distance"]="2000"
        args["eth2_bridged"]="1"
        args["eth2_multicast"]="0"
        args["eth2_nat"]="1"
        args["eth2_ipaddr"]="4"
        args["eth2_ipaddr_0"]="0"
        args["eth2_ipaddr_1"]="0"
        args["eth2_ipaddr_2"]="0"
        args["eth2_ipaddr_3"]="0"
        args["eth2_netmask"]="4"
        args["eth2_netmask_0"]="0"
        args["eth2_netmask_1"]="0"
        args["eth2_netmask_2"]="0"
        args["eth2_netmask_3"]="0"

        body = self.fetch(url, args)

    def current_ap(self):
        url = "http://%s/Status_Wireless.live.asp" % self.hostname
        body = self.fetch(url)
        
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

        print essid

    def site_survey(self):
        url = "http://%s/Site_Survey.asp" % self.hostname

        body = self.fetch(url)

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

        aps = {}
        
        fp = StringIO.StringIO(string.join(aplines, '\n'))
        reader = csv.reader(fp)
        for row in reader:
          essid = row[0]
          macattr = row[2]
          channel = int(row[3])
          rssi = int(row[4])
          noise = int(row[5])
          beacon = int(row[6])

          aps[essid] = row

        for ap, row in aps.items():
            print ap, row[8][:20]

import getopt
def main(argv, stdout, environ):
    progname = sys.argv[0]
    optlist, args = getopt.getopt(argv[1:], "", ["help"])
    
    dd = DDWRT("192.168.1.1", "root", "admin")

    if len(args) == 0:
        dd.current_ap()
        return
        
    cmd = args[0]
    if cmd == "connect":
        dd.connect(args[1])
    elif cmd == "survey":
        dd.site_survey()

if __name__ == "__main__":
    main(sys.argv, sys.stdout, os.environ)

