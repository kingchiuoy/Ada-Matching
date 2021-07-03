import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.config.Profile;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.ev.DefaultEncodedValueFactory;
import com.graphhopper.routing.util.DefaultFlagEncoderFactory;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.*;
import com.graphhopper.util.PMap;
import com.graphhopper.util.shapes.GHPoint;

import java.io.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.graphhopper.util.Parameters.Routing.MAX_VISITED_NODES;

public class Matching{
    public String length_s;
    public List<Observation> tracks;//保存轨迹数据
    public List<Observation> grounds;
    public double ScoreBound;
    public double Radius;

    private MapMatching matching;

    public Matching() {
        length_s = "";
        tracks = new ArrayList<>();
        grounds = new ArrayList<>();
    }

    public static void main(String[] args) throws IOException {
        Matching app = new Matching();

        String mapPath = "map-data/shanghai.osm.pbf";

        GraphHopperConfig graphHopperConfiguration = new GraphHopperConfig();
        String vehicle = "car";
        String ghFolder = "graph-cache";
        graphHopperConfiguration.putObject("graph.flag_encoders", vehicle);
        graphHopperConfiguration.putObject("datareader.file", mapPath);
        graphHopperConfiguration.putObject("graph.location", ghFolder);

        String weightingStr = "fastest";
        List<Profile> profiles = new ArrayList<>();
        for (String v : vehicle.split(",")) {
            v = v.trim();
            profiles.add(new Profile(v + "_profile").setVehicle(v).setWeighting(weightingStr));
        }
        graphHopperConfiguration.setProfiles(profiles);
        GraphHopper hopper = new GraphHopperOSM().init(graphHopperConfiguration);
        hopper.importOrLoad();

        if (Helper.isEmpty(vehicle))
            vehicle = EncodingManager.create(new DefaultEncodedValueFactory(), new DefaultFlagEncoderFactory(), ghFolder).fetchEdgeEncoders().get(0).toString();

        Profile profile = new Profile(vehicle + "_profile").setVehicle(vehicle).setWeighting(weightingStr).setTurnCosts(false);
        graphHopperConfiguration.setProfiles(Collections.singletonList(profile));
        hopper = new GraphHopperOSM().init(graphHopperConfiguration);
        hopper.importOrLoad();
        System.out.println("loading graph from cache");

        PMap hints = new PMap().putObject(MAX_VISITED_NODES, "3000");
        hints.putObject("profile", profile.getName());
        app.matching = new MapMatching(hopper, hints);

        app.Radius = 50;
        app.ScoreBound = 0.5;
        app.MatchProcess(0,61);
    }

    //读取文件
    private void readTrack(String file) {
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(file));
            String line = reader.readLine();
            line = reader.readLine();
            long stamp = Long.parseLong(line.split(",")[2]);//读取时间
            while (line != null) {
                String[] splits = line.split(",");
                long timestamp = (Long.parseLong(splits[2]) - stamp);//时间秒为单位
                double lat = Double.parseDouble(splits[3]);
                double lon = Double.parseDouble(splits[4]);
                double speed = Double.parseDouble(splits[5]);
                double direction = Double.parseDouble(splits[6]);
                double accuracy = Double.parseDouble(splits[7]);

                line = reader.readLine();
                tracks.add(new Observation(timestamp, lat, lon, speed, direction, accuracy));

            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    private void readGround(String file) {
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(file));
            String line = reader.readLine();
            line = reader.readLine();
            int num=0;
            while (line != null) {
                String[] splits = line.split(",");
                double lat = Double.parseDouble(splits[2]);
                double lon = Double.parseDouble(splits[3]);
                line = reader.readLine();
                grounds.add(new Observation(num, lat, lon, 0, 0, 0));
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    //匹配的输入输出封装
    private void MatchProcess(int method_num, int start_index, int end_index){
        String length_stat = "";
        matching.radius = Radius;
        matching.score_bound = ScoreBound;
        matching.method_choice = method_num;
        int[] count = new int[6];
        for (int i=start_index; i<end_index; i++){
            tracks.clear();grounds.clear();
            String file_num = String.valueOf(i);
            readTrack("map-data/third-data/track/" + file_num + ".csv");
            readGround("map-data/third-data/ground/" + file_num + ".csv");
            try {
                //匹配过程
                MatchResult matchpath =  matching.matchWithSpecificMethod(tracks, ScoreBound, method_num);
                MatchResult hmmpath =  matching.match(tracks);
                //计算与ground truth的精度差
                double gpxLength = matching.gpxLength(grounds);length_s += gpxLength + "\n";
                double matchLength = matchpath.getMatchLength();
                double hmmLength = hmmpath.getMatchLength();
                double match_e = (matchLength - gpxLength) / gpxLength;
                double hmm_e = (hmmLength - gpxLength) / gpxLength;
                if (Math.abs(match_e)<=0.05 && Math.abs(hmm_e)>=0.15) System.out.println("序号"+ i + " 误差" + match_e + " 误差值" + (matchLength-gpxLength));
                /*if (match_e>-0.1 && match_e <=-0.05) count[0]++;
                else if (match_e>-0.05 && match_e<=0) count[1]++;
                else if (match_e>0 && match_e<=0.05) count[2]++;
                else if (match_e>0.05 && match_e<0.1) count[3]++;*/
                double value = Math.abs(match_e);
                if (Math.abs(match_e) <=0.05 ||((matchLength-gpxLength)>-400 && (matchLength<gpxLength))) count[0]++;
                //if (value <=0.05) count[0]++;
                else if (value <= 0.1) count[1]++;
                else if (value <= 0.15) count[2]++;
                else if (value <= 0.2) count[3]++;
                else if (value <= 0.25) count[4]++;
                else if (value <= 0.3) count[5]++;
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            }
        }
        String str_tmp = "-0.1~-0.05,-0.05~0,0~0.05,0.05~0.1\n";
        for (Integer number : count) {
            str_tmp += number + ",";
            System.out.print(number+" ");
        }
        str_tmp += '\n';
        String method;
        if (method_num == 0) method = "Union";
        else if (method_num == 1) method = "Ada";
        else method = "Reverse";
        File Parameters = new File("map-data/"+method+"MatchCount.txt");
        File Length = new File("map-data/MatchLength.txt");
        try{
            FileOutputStream fos = new FileOutputStream(Parameters);
            OutputStreamWriter dos = new OutputStreamWriter(fos);
            FileOutputStream ff = new FileOutputStream(Length);
            OutputStreamWriter dd = new OutputStreamWriter(ff);
            try {
                dos.write(str_tmp);dos.close();
                dd.write(length_s);dd.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    private void MatchProcess(int method_num, int index){
        matching.radius = Radius;
        matching.score_bound = ScoreBound;
        matching.method_choice = method_num;
        tracks.clear();grounds.clear();
        String file_num = String.valueOf(index);
        readTrack("map-data/third-data/track/" + file_num + ".csv");
        readGround("map-data/third-data/ground/" + file_num + ".csv");
        try {
            //匹配过程
            StopWatch stopwatch = new StopWatch();
            stopwatch.start();
            MatchResult AmmPath =  matching.matchWithSpecificMethod(tracks, ScoreBound, method_num);
            MatchResult HmmPath =  matching.match(tracks);
            stopwatch.stop();
            //System.out.println(stopwatch);
            //计算与ground truth的精度差
            double gpxLength = matching.gpxLength(grounds);
            double AmmLength = AmmPath.getMatchLength();
            double HmmLength = HmmPath.getMatchLength();
            double amm_e = (AmmLength - gpxLength) / gpxLength;
            double hmm_e = (HmmLength - gpxLength) / gpxLength;
            System.out.println("AMM 序号"+ index + " 误差" + amm_e + " 误差值" + (AmmLength-gpxLength));
            System.out.println("HMM 序号"+ index + " 误差" + amm_e + " 误差值" + (AmmLength-gpxLength));
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
        }
    }
}