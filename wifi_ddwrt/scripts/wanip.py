#! /usr/bin/env python

"""
usage: %(progname)s hostname
"""

import os, sys, string, time, getopt, re, urllib

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", [])

  testflag = 0
  if len(args) == 0:
    usage(progname)
    return

  host = args[0]

  url = "http://%s/" % host
  fp = urllib.urlopen(url)
  body = fp.read()
  fp.close()

  body = re.sub('<[-=+/"A-Za-z0-9 ]*>', "", body)
  body = re.sub("&nbsp;", " ", body)

  pat = re.compile("WAN IP: ([0-9.]*)")
  m = pat.search(body)
  if m:
    wanip = m.group(1)
    print wanip
    return 0
  return 1

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
