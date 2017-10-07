#!/usr/bin/python

import Channel

channel = Channel.Channel( "TV1 TV2 ChannelA ChannelB Channel58 ChX" )

for ch in channel:
    print( ch )
        
