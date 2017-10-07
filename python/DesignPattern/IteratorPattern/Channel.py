#!/usr/bin/python

import ChannelIterator

class Channel:
    def __init__( self, channels ):
        self.channels = channels

    def __iter__( self ):
        return ChannelIterator.ChannelIterator( self.channels )
