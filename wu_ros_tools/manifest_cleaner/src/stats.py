#!/usr/bin/python

import roslib; roslib.load_manifest('manifest_cleaner')
import rospy
import xml.dom.minidom
import os
import os.path
import sys
import collections

authors = collections.defaultdict(list)

for root, subFolders, files in os.walk(sys.argv[1]):
    if 'manifest.xml' not in files:
        continue
    package = os.path.basename(root)
    manifest_xml = open("%s/manifest.xml"%root, 'r').read()
    manifest = xml.dom.minidom.parseString(manifest_xml)
    author = manifest.getElementsByTagName('author')[0].childNodes[0].data
    authors[author].append(package)

for a,c in sorted(authors.items()):
    print "%s %4d"%(a.strip(),len(c))
    for b in c:
        print '\t%s'%b
