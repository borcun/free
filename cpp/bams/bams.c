/**
 * @source : https://wilke.de/uploads/media/REAL_TO_BAM_Conversion_03.tig
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* bam bit table */
double bam_bit_table[ 16 ] = { 0.0055, 0.0109, 0.0219, 0.0439, 0.088, 0.1757, 0.3515, 0.703, 1.406, 2.8125, 5.625, 11.25, 22.5, 45.0, 90.0, 180.0 };

void print2base( int num, int count ) {
  if( 0 == num ) {
    printf( "[%d] 0 ", ++count );
    return;
  }
  else if( 1 == num || -1 == num ) {
    printf( "[%d] 1 ", ++count );
    return;
  }

  print2base( num / 2, ++count );
  printf( "%d ", abs(num) % 2 );
}

unsigned short realToUBAM( const double _num ) {
  double num = _num;
  unsigned short res = 0;
  int i;
  
  if( num == 360.0 )
    return 0.0;

  for( i=15 ; i >=0 ; --i ) {
    if( num >= bam_bit_table[ i ] ) {
      num -= bam_bit_table[ i ];
      res += pow( 2, i );
    }
  }
  return res;
}

signed realToBAM( const double _num ) {
  double num = _num;
  signed short res = 0;
  int i;
  
  if( num == 360.0 )
    return 0.0;
  
  for( i=15 ; i >= 0 ; --i ) {
    if( fabs( num ) >= bam_bit_table[ i ] ) {
      num = fabs( num ) - bam_bit_table[ i ];
      res += pow( 2, i );
    }
  }
  return res; 
}

int main( int argc, char **argv ) {
  double real = atof( argv[1] );
  unsigned short ubam;
  short bam;
  
  ubam = realToUBAM( real );
  bam = realToBAM( real );

  printf( "%f degree = %d UBAM = %d BAM = ", real, ubam, bam );
  print2base( bam, 0 );
  printf( "binary\n" );
  
  return 0;
}
