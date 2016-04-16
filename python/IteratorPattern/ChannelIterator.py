#!/usr/bin/python

class ChannelIterator:
    def __init__( self, channels ):
        self.channels = [ channel for channel in channels.split() ]
        self.index = 0

    def __next__( self ):
        if self.index == len( self.channels ):
            raise StopIteration()

        channel = self.channels[ self.index ]
        self.index += 1

        return channel

    def __iter__( self ):
        return self
