import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import org.json.simple.parser.ParseException;
import org.json.simple.parser.JSONParser;

/*
 
  {"coord":{"lon":32.85,"lat":39.92},"sys":{"message":0.0081,"country":"TR","sunrise":1434680385,"sunset":1434734410},"weather":[{"id":802,"main":"Clouds","description":"scattered clouds","icon":"03d"}],"base":"stations","main":{"temp":292.64,"temp_min":292.64,"temp_max":292.64,"pressure":887.94,"sea_level":1016.6,"grnd_level":887.94,"humidity":85},"wind":{"speed":1.66,"deg":273.505},"clouds":{"all":48},"dt":1434716561,"id":323786,"name":"Ankara","cod":200}
  
  * */

class JSON 
{
   public static void main(String[] args) 
   {
      JSONParser parser=new JSONParser();
      String s = "[{\"coord\":{\"lon\":32.85,\"lan\":39.92}}]";
      try{
         Object obj = parser.parse(s);
         JSONArray array = (JSONArray)obj;
         System.out.println("The 1st element of array");
         System.out.println(array.get(0));
         System.out.println();

         
         JSONObject obj2 = (JSONObject)array.get(0);
         System.out.println("Field \"2\"");
         System.out.println(obj2.get("coord"));    

         JSONObject obj3 = obj2;
         System.out.println("Field \"3\"");
         System.out.println( obj3 );    
        
      }catch(ParseException pe){
         System.out.println("position: " + pe.getPosition());
         System.out.println(pe);
      }
   }
}