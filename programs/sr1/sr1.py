#!/usr/bin/env python

# 2013 (c) edits by Santiago Morante and Juan G Victores, 

# Copyright (c) 2008 Carnegie Mellon University.
#
# You may modify and redistribute this file under the same terms as
# the CMU Sphinx system.  See
# http://cmusphinx.sourceforge.net/html/LICENSE for more information.

import pygtk
pygtk.require('2.0')
import gtk

import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()
import gst

import yarp
import os.path

class DemoApp(object):
    """GStreamer/PocketSphinx Demo Application"""
    def __init__(self):
        """Initialize a DemoApp object"""
        self.models_dir = os.path.join(os.environ['TEO_HEAD_ROOT'],'app','asr','models','default')
        self.outPort = yarp.Port()
        self.outPort.open('/asr:o')
        self.init_gst()

    def init_gst(self):
        """Initialize the speech components"""
        self.pipeline = gst.parse_launch('gconfaudiosrc ! audioconvert ! audioresample '
                                         + '! vader name=vad auto-threshold=true '
                                         + '! pocketsphinx name=asr ! fakesink')
        asr = self.pipeline.get_by_name('asr')
        asr.connect('result', self.asr_result)
        asr.set_property('lm', os.path.join(self.models_dir,'model5.lm') )
        asr.set_property('dict', os.path.join(self.models_dir,'dict5.dic') )
        asr.set_property('configured', True)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()

        self.pipeline.set_state(gst.STATE_PLAYING)

    def asr_result(self, asr, text, uttid):
        """Forward result signals on the bus to the main thread."""
        print '---'
        b = yarp.Bottle()
        #s = text.lower().split()
        #for elem in s:
        #    b.addString(elem)
        #    print elem
        #if b.size() != 0:
        #    self.outPort.write(b)
        print text.lower()
        b.addString(text.lower())
        if text != "":
            self.outPort.write(b)

yarp.Network.init()
if yarp.Network.checkNetwork() != True:
    print '[asr] error: found no yarp network (try running "yarpserver &"), bye!'
    quit()

app = DemoApp()
gtk.main()
