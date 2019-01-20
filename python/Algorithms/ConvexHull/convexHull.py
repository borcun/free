#!/usr/bin/python3

# 2D points class
class Point:
    def __init__( self ):
        self.x = 0
        self.y = 0

    def __init__( self, x, y ):
        self.x = x
        self.y = y

    def matchX( self, p1, p2 ):
        return self._match( p1.x > p2.x )
    
    def matchY( self, p1, p2 ):
        return self._match( p1.y > p2.y )

    def _match( self, c1, c2 ):
        if c1 > c2:
            return 1
        elif c1 < c2:
            return -1
        else:
            return 0
        

class ConvexHull:
    def __init__( self ):
        self.points = []

    def readFile( self, file_path ):
        self.points = []

        f = open( file_path, 'r' )

        for line in f:
            tp = line.rstrip().split( "," )
            p = Point( int( tp[0] ), int( tp[1] ) )

            self.points.append( p )

        f.close()

        print( self.points )
        
    def solve( self ):
        if 3 > len( self.points ):
            print( " Point counts are not enough to solve!\n" )
        elif 0 == len( self.points ):
            print( " Please fill the points before!\n" )
        else:
            print( "solving" )

        return
