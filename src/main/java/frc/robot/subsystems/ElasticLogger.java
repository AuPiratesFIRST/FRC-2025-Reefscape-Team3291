// package frc.robot.subsystems;

// import java.io.OutputStream;
// import java.net.HttpURLConnection;
// import java.net.URL;
// import org.json.simple.JSONObject;

// public class ElasticLogger {
//     private static final String ELASTIC_URL = "http://10.32.91.200:9200/limelight-data/_doc"; // Change this to your Elastic instance

//     public static void sendToElastic(JSONObject data) {
//         try {
//             URL url = new URL(ELASTIC_URL);
//             HttpURLConnection connection = (HttpURLConnection) url.openConnection();
//             connection.setRequestMethod("POST");
//             connection.setRequestProperty("Content-Type", "application/json");
//             connection.setDoOutput(true);

//             try (OutputStream os = connection.getOutputStream()) {
//                 byte[] input = data.toString().getBytes("utf-8");
//                 os.write(input, 0, input.length);
//             }

//             int responseCode = connection.getResponseCode();
//             System.out.println("Elastic Response Code: " + responseCode);
//         } catch (Exception e) {
//             e.printStackTrace();
//         }
//     }
// }