public class InsertionSort {
	public static void swap( int[] arr, int i, int j ) {
		int temp = arr[i];
		arr[i] = arr[j];
		arr[j] = temp;

		return;
	}

	public static void insertionSort( int[] arr ) {
		for( int i=1 ; i < arr.length ; ++i ) {
			for( int j=i ; j > 0 && arr[j-1] > arr[j] ; --j )
					swap( arr, j, j-1 );
		}

		return;
	}

	public static void main( String[] args ) {
		int[] arr = new int[6];

		arr[0] = 4;
		arr[1] = 3;
		arr[2] = 7;
		arr[3] = 1;
		arr[4] = 5;
		arr[5] = 6;

		System.out.print("before sorting : ");
		for( int i=0 ; i < arr.length ; ++i )
			System.out.printf("%d ", arr[i] );

		System.out.println();
		insertionSort( arr );

		System.out.print("after sorting : ");
		for( int i=0 ; i < arr.length ; ++i )
			System.out.printf("%d ", arr[i] );

		System.out.println();
	}
}
