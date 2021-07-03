package com.graphhopper.matching.cli;

import com.fasterxml.jackson.dataformat.xml.XmlMapper;
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
import io.dropwizard.cli.Command;
import io.dropwizard.setup.Bootstrap;
import net.sourceforge.argparse4j.inf.Namespace;
import net.sourceforge.argparse4j.inf.Subparser;

import java.io.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.graphhopper.util.Parameters.Routing.MAX_VISITED_NODES;

public class AdaMatchCommand extends Command {

    public AdaMatchCommand() {
        super("adamatch", "map-match with Ada-Matching for one or more csv files");
    }

    public List<Observation> tracks;

    @Override
    public void configure(Subparser subparser) {
        subparser.addArgument("csv")
                .type(File.class)
                .required(true)
                .nargs("+")
                .help("GPX file");

    }

    @Override
    public void run(Bootstrap bootstrap, Namespace args) {
        tracks = new ArrayList<>();
        GraphHopperConfig graphHopperConfiguration = new GraphHopperConfig();
        String ghFolder = "graph-cache";
        graphHopperConfiguration.putObject("graph.location", ghFolder);

        String vehicle = "car";
        if (Helper.isEmpty(vehicle))
            vehicle = EncodingManager.create(new DefaultEncodedValueFactory(), new DefaultFlagEncoderFactory(), ghFolder).fetchEdgeEncoders().get(0).toString();
        // Penalizing inner-link U-turns only works with fastest weighting, since
        // shortest weighting does not apply penalties to unfavored virtual edges.
        String weightingStr = "fastest";
        Profile profile = new Profile(vehicle + "_profile").setVehicle(vehicle).setWeighting(weightingStr).setTurnCosts(false);
        graphHopperConfiguration.setProfiles(Collections.singletonList(profile));
        GraphHopper hopper = new GraphHopperOSM().init(graphHopperConfiguration);
//        System.out.println("loading graph from cache");
        hopper.load(graphHopperConfiguration.getString("graph.location", ghFolder));

        PMap hints = new PMap().putObject(MAX_VISITED_NODES, "3000");
        hints.putObject("profile", profile.getName());
        MapMatching mapMatching = new MapMatching(hopper, hints);

//        Set the two parameters
        //mapMatching.setTransitionProbabilityBeta(2.0);
        mapMatching.setRadius(50);


        StopWatch importSW = new StopWatch();
        StopWatch matchSW = new StopWatch();

        Translation tr = new TranslationMap().doImport().getWithFallBack(Helper.getLocale("instructions"));//args.getString("instructions")));
        final boolean withRoute = "instruction".isEmpty();//!args.getString("instructions").isEmpty();
        XmlMapper xmlMapper = new XmlMapper();
        Weighting weighting = hopper.createWeighting(hopper.getProfiles().get(0), hints);

        mapMatching.setRadius(25);

        String out = "map-data/result.txt";

        try {
            FileWriter fwriter = new FileWriter(out);
            fwriter.write("");
            fwriter.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        int i = 0;
        for (File csvFile : args.<File>getList("csv")) {
            String str = String.format("%04d", i);
            String tail = str + ".csv";
            matchSW.start();
            tracks.clear();
            readFile(csvFile);
                try {
                    importSW.start();
                    importSW.stop();

                    MatchResult mr = mapMatching.matchWithSpecificMethod(tracks, 0.7, 1);

                    matchSW.stop();
                    try {
                        FileWriter fwriter = new FileWriter(out, true);
                        double groundLength = mr.getGpxEntriesLength();
                        double matchLength = mr.getMatchLength();
                        double tmp = Math.min(1.0, Math.abs(groundLength - matchLength) / groundLength);
                        double rate = 1.0 - tmp;
                        fwriter.write(i + "," + mr.getMatchLength() + ","
                                + groundLength + "," + rate);
                        fwriter.write("\n");
                        fwriter.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                } catch (IllegalArgumentException e) {
                    e.printStackTrace();
                    System.out.println("The" + i + "Failed");
                }
                i++;
        }
    }

    public  void readFile(File file) {
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(
                    file
            ));
            String line = reader.readLine();
            line = reader.readLine();
//            System.out.println(line);
            while (line != null) {
                String[] splits = line.split(",");
                long timestamp = Long.parseLong(splits[1]) / 1000;
                double lat = Double.parseDouble(splits[2]);
                double lon = Double.parseDouble(splits[3]);
                double speed = Double.parseDouble(splits[4]);
                double direction = Double.parseDouble(splits[5]);
                double acc = Double.parseDouble(splits[6]);
                tracks.add(new Observation(timestamp, lat, lon, speed, direction, acc));

//                System.out.println("Time" + timestamp + " " + lat + " " + lon + " " + speed);
                line = reader.readLine();
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
