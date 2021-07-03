import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.config.LMProfile;
import com.graphhopper.config.Profile;
import com.graphhopper.matching.*;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.AStarBidirection;
import com.graphhopper.routing.BidirRoutingAlgorithm;
import com.graphhopper.routing.DijkstraBidirectionRef;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.DefaultEncodedValueFactory;
import com.graphhopper.routing.lm.LMApproximator;
import com.graphhopper.routing.lm.LandmarkStorage;
import com.graphhopper.routing.lm.PrepareLandmarks;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.DefaultFlagEncoderFactory;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;
import com.graphhopper.util.shapes.GHPoint3D;

import java.util.*;
import java.util.stream.Collectors;

import static com.graphhopper.util.Parameters.Routing.MAX_VISITED_NODES;

public class CandidateTest {

    public final Graph graph;
    private final PrepareLandmarks landmarks;
    private final LocationIndexTree locationIndex;
    private double measurementErrorSigma = 50.0;
    private double transitionProbabilityBeta = 2.0;
    private final int maxVisitedNodes;
    private final DistanceCalc distanceCalc = new DistancePlaneProjection();
    private final Weighting weighting;
    public QueryGraph queryGraph;


    public CandidateTest(GraphHopper graphHopper, PMap hints) {
        this.locationIndex = (LocationIndexTree) graphHopper.getLocationIndex();

        if (hints.has("vehicle"))
            throw new IllegalArgumentException("MapMatching hints may no longer contain a vehicle, use the profile parameter instead, see core/#1958");
        if (hints.has("weighting"))
            throw new IllegalArgumentException("MapMatching hints may no longer contain a weighting, use the profile parameter instead, see core/#1958");

        if (graphHopper.getProfiles().isEmpty()) {
            throw new IllegalArgumentException("No profiles found, you need to configure at least one profile to use map matching");
        }
        if (!hints.has("profile")) {
            throw new IllegalArgumentException("You need to specify a profile to perform map matching");
        }
        String profileStr = hints.getString("profile", "");
        Profile profile = graphHopper.getProfile(profileStr);
        if (profile == null) {
            List<Profile> profiles = graphHopper.getProfiles();
            List<String> profileNames = new ArrayList<>(profiles.size());
            for (Profile p : profiles) {
                profileNames.add(p.getName());
            }
            throw new IllegalArgumentException("Could not find profile '" + profileStr + "', choose one of: " + profileNames);
        }

        boolean disableLM = hints.getBool(Parameters.Landmark.DISABLE, false);
        if (graphHopper.getLMPreparationHandler().isEnabled() && disableLM && !graphHopper.getRouterConfig().isLMDisablingAllowed())
            throw new IllegalArgumentException("Disabling LM is not allowed");

        boolean disableCH = hints.getBool(Parameters.CH.DISABLE, false);
        if (graphHopper.getCHPreparationHandler().isEnabled() && disableCH && !graphHopper.getRouterConfig().isCHDisablingAllowed())
            throw new IllegalArgumentException("Disabling CH is not allowed");

        // see map-matching/#177: both ch.disable and lm.disable can be used to force Dijkstra which is the better
        // (=faster) choice when the observations are close to each other
        boolean useDijkstra = disableLM || disableCH;

        if (graphHopper.getLMPreparationHandler().isEnabled() && !useDijkstra) {
            // using LM because u-turn prevention does not work properly with (node-based) CH
            List<String> lmProfileNames = new ArrayList<>();
            PrepareLandmarks lmPreparation = null;
            for (LMProfile lmProfile : graphHopper.getLMPreparationHandler().getLMProfiles()) {
                lmProfileNames.add(lmProfile.getProfile());
                if (lmProfile.getProfile().equals(profile.getName())) {
                    lmPreparation = graphHopper.getLMPreparationHandler().getPreparation(
                            lmProfile.usesOtherPreparation() ? lmProfile.getPreparationProfile() : lmProfile.getProfile()
                    );
                }
            }
            if (lmPreparation == null) {
                throw new IllegalArgumentException("Cannot find LM preparation for the requested profile: '" + profile.getName() + "'" +
                        "\nYou can try disabling LM using " + Parameters.Landmark.DISABLE + "=true" +
                        "\navailable LM profiles: " + lmProfileNames);
            }
            landmarks = lmPreparation;
        } else {
            landmarks = null;
        }
        graph = graphHopper.getGraphHopperStorage();
        weighting = graphHopper.createWeighting(profile, hints);
        this.maxVisitedNodes = hints.getInt(Parameters.Routing.MAX_VISITED_NODES, Integer.MAX_VALUE);
    }


    public void init() {
        List<Observation> o = new ArrayList<>();
        Observation o1 = new Observation(1.375441,103.955197);
        Observation o2 = new Observation(1.3756389999999998,103.9561);
        o.add(o1);o.add(o2);

        List<Collection<Snap>> splitsPerObservation = o.stream().map(l -> locationIndex.findNClosest(l.getPoint().lat, l.getPoint().lon, DefaultEdgeFilter.allEdges(weighting.getFlagEncoder()), 50)).collect(Collectors.toList());
        queryGraph = QueryGraph.create(graph, splitsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));

        splitsPerObservation = splitsPerObservation.stream().map(this::deduplicate).collect(Collectors.toList());

        List<ObservationWithCandidateStates> timeSteps = createTimeSteps(o, splitsPerObservation);

        for (State from : timeSteps.get(0).candidates){
            for (State to : timeSteps.get(1).candidates){
                final Path path = createRouter().calcPath(from.getSnap().getClosestNode(), to.getSnap().getClosestNode(),
                        from.isOnDirectedEdge() ? from.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                        to.isOnDirectedEdge() ? to.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);
                if (path.isFound()) {
                    System.out.println(path.getDistance());
                    for (GHPoint3D point : path.calcPoints()){
                        System.out.println(point.lat + "," + point.lon);
                    }
//                    if (from.isOnDirectedEdge())
//                        System.out.println("yes");
                }
                System.out.println("one path");
            }
        }
    }
    private Collection<Snap> deduplicate(Collection<Snap> splits) {
        // Only keep one split per node number. Let's say the last one.
        Map<Integer, Snap> splitsByNodeNumber = splits.stream().collect(Collectors.toMap(Snap::getClosestNode, s -> s, (s1, s2) -> s2));
        return splitsByNodeNumber.values();
    }
    private List<ObservationWithCandidateStates> createTimeSteps(List<Observation> filteredObservations, List<Collection<Snap>> splitsPerObservation) {
        if (splitsPerObservation.size() != filteredObservations.size()) {
            throw new IllegalArgumentException(
                    "filteredGPXEntries and queriesPerEntry must have same size.");
        }

        final List<ObservationWithCandidateStates> timeSteps = new ArrayList<>();
        for (int i = 0; i < filteredObservations.size(); i++) {
            Observation observation = filteredObservations.get(i);
            Collection<Snap> splits = splitsPerObservation.get(i);
            List<State> candidates = new ArrayList<>();
            for (Snap split : splits) {
                if (queryGraph.isVirtualNode(split.getClosestNode())) {
                    List<VirtualEdgeIteratorState> virtualEdges = new ArrayList<>();
                    EdgeIterator iter = queryGraph.createEdgeExplorer().setBaseNode(split.getClosestNode());
                    while (iter.next()) {
                        if (!queryGraph.isVirtualEdge(iter.getEdge())) {
                            throw new RuntimeException("Virtual nodes must only have virtual edges "
                                    + "to adjacent nodes.");
                        }
                        virtualEdges.add((VirtualEdgeIteratorState) queryGraph.getEdgeIteratorState(iter.getEdge(), iter.getAdjNode()));
                    }
                    if (virtualEdges.size() != 2) {
                        throw new RuntimeException("Each virtual node must have exactly 2 "
                                + "virtual edges (reverse virtual edges are not returned by the "
                                + "EdgeIterator");
                    }

                    // Create a directed candidate for each of the two possible directions through
                    // the virtual node. We need to add candidates for both directions because
                    // we don't know yet which is the correct one. This will be figured
                    // out by the Viterbi algorithm.
                    candidates.add(new State(observation, split, virtualEdges.get(0), virtualEdges.get(1)));
                    candidates.add(new State(observation, split, virtualEdges.get(1), virtualEdges.get(0)));
                } else {
                    // Create an undirected candidate for the real node.
                    candidates.add(new State(observation, split));
                }
            }

            timeSteps.add(new ObservationWithCandidateStates(observation, candidates));
        }
        return timeSteps;
    }

    public static void main(String[] args) {

        String mapPath = "map-data/singapore-latest.osm.pbf";
        GraphHopperConfig graphHopperConfiguration = new GraphHopperConfig();
        String vehicle = "car";
        String ghFolder = "graph-cache";
        String out = "map-data/t.txt";
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
        // Penalizing inner-link U-turns only works with fastest weighting, since
        // shortest weighting does not apply penalties to unfavored virtual edges.
        Profile profile = new Profile(vehicle + "_profile").setVehicle(vehicle).setWeighting(weightingStr).setTurnCosts(false);
        graphHopperConfiguration.setProfiles(Collections.singletonList(profile));
        hopper = new GraphHopperOSM().init(graphHopperConfiguration);
        hopper.importOrLoad();
        System.out.println("loading graph from cache");
//        hopper.load(graphHopperConfiguration.getString("graph.location", ghFolder));

        PMap hints = new PMap().putObject(MAX_VISITED_NODES, "3000");
        hints.putObject("profile", profile.getName());

        CandidateTest test = new CandidateTest(hopper, hints);
        test.init();
    }
    private BidirRoutingAlgorithm createRouter() {
        BidirRoutingAlgorithm router;
        if (landmarks != null) {
            //System.out.println("Not");
            AStarBidirection algo = new AStarBidirection(queryGraph, weighting, TraversalMode.EDGE_BASED) {
                @Override
                protected void initCollections(int size) {
                    super.initCollections(50);
                }
            };
            LandmarkStorage lms = landmarks.getLandmarkStorage();
            int activeLM = Math.min(8, lms.getLandmarkCount());
            algo.setApproximation(LMApproximator.forLandmarks(queryGraph, lms, activeLM));
            algo.setMaxVisitedNodes(maxVisitedNodes);
            router = algo;
        } else {
            router = new DijkstraBidirectionRef(queryGraph, weighting, TraversalMode.EDGE_BASED) {
                @Override
                protected void initCollections(int size) {
                    super.initCollections(50);
                }
            };
            router.setMaxVisitedNodes(maxVisitedNodes);
        }
        return router;
    }
    static List<String> fetchStreets(List<EdgeMatch> emList) {
        List<String> list = new ArrayList<>();
        int prevNode = -1;
        List<String> errors = new ArrayList<>();
        for (EdgeMatch em : emList) {
            String str = em.getEdgeState().getName();// + ":" + em.getEdgeState().getBaseNode() +
            // "->" + em.getEdgeState().getAdjNode();
            list.add(str);
            if (prevNode >= 0) {
                if (em.getEdgeState().getBaseNode() != prevNode) {
                    errors.add(str);
                }
            }
            prevNode = em.getEdgeState().getAdjNode();
        }

        if (!errors.isEmpty()) {
            throw new IllegalStateException("Errors:" + errors);
        }
        return list;
    }
}
