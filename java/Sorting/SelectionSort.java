public class SelectionSort {
	public static void swap( int[] arr, int i, int j ) {
		int temp = arr[i];
		arr[i] = arr[j];
		arr[j] = temp;

		return;
	}

	public static void selectionSort( int[] arr ) {
		for( int i=0 ; i < arr.length - 1 ; ++i ) {
			int min_index = i;

			for(int j=i+1 ; j < arr.length ; ++j ) {
				if( arr[ min_index ] > arr[j] )
					min_index = j;
			}

			swap( arr, i, min_index );
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
		selectionSort( arr );

		System.out.print("after sorting : ");
		for( int i=0 ; i < arr.length ; ++i )
			System.out.printf("%d ", arr[i] );

		System.out.println();
	}
}
