import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

public class Length {
    public String length_s;
    public List<Observation> grounds;
    public Length(){
        length_s = "";
        grounds = new ArrayList<>();
    }
    public static void main(String[] args) throws IOException {
        Length length = new Length();
        length.Process(1,200);

    }
    private MapMatching matching;

    private void readGround(String file) {
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(file));
            String line = reader.readLine();
            line = reader.readLine();
            int num=0;
            while (line != null) {
                String[] splits = line.split(",");
                double lat = Double.parseDouble(splits[3]);
                double lon = Double.parseDouble(splits[4]);
                line = reader.readLine();
                grounds.add(new Observation(num, lat, lon, 0, 0, 0));
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void Process(int start_index, int end_index){
        int[] count = new int[6];
        for (int i=start_index; i<end_index; i++){
            String file_num = String.valueOf(i);
            readGround("map-data/singapore/track/" + file_num + ".csv");
            try {
                //计算与ground truth的精度差
                double gpxLength = matching.gpxLength(grounds);
                length_s += gpxLength + "\n";
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            }
        }
        File Parameters = new File("map-data/MatchLength.txt");
        try{
            FileOutputStream fos = new FileOutputStream(Parameters);
            OutputStreamWriter dos = new OutputStreamWriter(fos);
            try {
                dos.write(length_s);dos.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}
