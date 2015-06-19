import org.json.simple.JSONObject;

public class JSON {
	public static void main( String[] args ) {
		String sURL = "http://api.openweathermap.org/data/2.5/weather?q=Ankara,tr";

    // Connect to the URL using java's native library
    URL url = new URL( sURL );
    HttpURLConnection request = (HttpURLConnection) url.openConnection();
    request.connect();

    // Convert to a JSON object to print data
    JSONParser jp = new JSONParser(); //from gson
    JSONElement root = jp.parse(new InputStreamReader((InputStream) request.getContent())); 
		//convert the input stream to a json element
    JsonObject rootobj = root.getAsJsonObject(); //may be an array, may be an object. 
    String zipcode = rootobj.get("lon").getAsString();//just grab the zipcode

		System.out.println( zipcode );
	}
}
