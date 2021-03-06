package com.graphhopper.matching;

import com.bmw.hmm.SequenceState;
import com.bmw.hmm.Transition;
import com.bmw.hmm.ViterbiAlgorithm;
import com.graphhopper.GraphHopper;
import com.graphhopper.config.LMProfile;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.*;
import com.graphhopper.routing.lm.LMApproximator;
import com.graphhopper.routing.lm.LandmarkStorage;
import com.graphhopper.routing.lm.PrepareLandmarks;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;
import com.graphhopper.util.shapes.GHPoint3D;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;

import java.io.*;
import java.util.*;
import java.util.stream.Collectors;

/**
 * This class matches real world GPX entries to the digital road network stored
 * in GraphHopper. The Viterbi algorithm is used to compute the most likely
 * sequence of map matching candidates. The Viterbi algorithm takes into account
 * the distance between GPX entries and map matching candidates as well as the
 * routing distances between consecutive map matching candidates.
 * <p>
 * <p>
 * See http://en.wikipedia.org/wiki/Map_matching and Newson, Paul, and John
 * Krumm. "Hidden Markov map matching through noise and sparseness." Proceedings
 * of the 17th ACM SIGSPATIAL International Conference on Advances in Geographic
 * Information Systems. ACM, 2009.
 *
 * @author Peter Karich
 * @author Michael Zilske
 * @author Stefan Holder
 * @author kodonnell
 */
public class MapMatching {

    private final Logger logger = LoggerFactory.getLogger(getClass());
    private final Graph graph;
    private final PrepareLandmarks landmarks;
    private final LocationIndexTree locationIndex;
    private final int maxVisitedNodes;
    private final DistanceCalc distanceCalc = new DistancePlaneProjection();
    private final Weighting weighting;
    private QueryGraph queryGraph;

    private final double sigma_theta = 30;
    private final double sigma_z = 40;
    private final List<Double> velocity = new ArrayList<>();
    private double v0;

    public double radius;
    public double score_bound;
    public int method_choice;

    public int total_delete=0;//????????????????????????

    //MapMatching????????????
    public MapMatching(GraphHopper graphHopper, PMap hints) {
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

//        Reduce the search radius for quicker calculation.
        int myMaxNodes = 3000;

//        this.maxVisitedNodes = hints.getInt(Parameters.Routing.MAX_VISITED_NODES, Integer.MAX_VALUE);
        this.maxVisitedNodes = hints.getInt(Parameters.Routing.MAX_VISITED_NODES, myMaxNodes);
    }

    //???????????????????????????
    public void setRadius(double Radius) {
        this.radius = Radius;
    }

    //??????????????????????????????
    private List<Observation> filterObservations(List<Observation> observations) {
        List<Observation> filtered = new ArrayList<>();
        Observation prevEntry = null;
        int last = observations.size() - 1;
        for (int i = 0; i <= last; i++) {
            Observation observation = observations.get(i);
            if (i == 0 || i == last || distanceCalc.calcDist(
                    prevEntry.getPoint().getLat(), prevEntry.getPoint().getLon(),
                    observation.getPoint().getLat(), observation.getPoint().getLon()) > 2 * radius) {
                filtered.add(observation);
                prevEntry = observation;
            } else {
                logger.debug("Filter out observation: {}", i + 1);
            }
        }
        return filtered;
    }

    private Collection<Snap> deduplicate(Collection<Snap> splits) {
        // Only keep one split per node number. Let's say the last one.
        Map<Integer, Snap> splitsByNodeNumber = splits.stream().collect(Collectors.toMap(Snap::getClosestNode, s -> s, (s1, s2) -> s2));
        return splitsByNodeNumber.values();
    }

    //??????????????????????????????+????????????????????????
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

    //???????????????????????????
    public MatchResult match(List<Observation> observations) {
        List<Observation> filteredObservations = filterObservations(observations);
        List<Collection<Snap>> splitsPerObservation = filteredObservations.stream().map(o -> locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(weighting.getFlagEncoder()), radius)).collect(Collectors.toList());
        queryGraph = QueryGraph.create(graph, splitsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        splitsPerObservation = splitsPerObservation.stream().map(this::deduplicate).collect(Collectors.toList());
        List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, splitsPerObservation);

        // HMM??????
        List<SequenceState<State, Observation, Path>> seq = computeViterbiSequence(timeSteps);
        String in_observ = "", in_candid = "", in_origin = "", in_result = "", in_path = "";//?????????????????????
        List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.path != null).flatMap(s1 -> s1.path.calcEdges().stream()).collect(Collectors.toList());
        for (EdgeIteratorState sequence : path){
            PointList short_path = sequence.fetchWayGeometry(FetchMode.ALL);
            for (GHPoint3D point : short_path) in_path += point.lat + "," + point.lon + '\n';
        }
        WriteFile(in_observ, in_origin, in_candid, in_result, in_path, "HMM");
        MatchResult result = new MatchResult(prepareEdgeMatches(seq));
        result.setMergedPath(new MapMatchedPath(queryGraph, weighting, path));
        result.setMatchMillis(seq.stream().filter(s -> s.path != null).mapToLong(s -> s.path.getTime()).sum());
        result.setMatchLength(seq.stream().filter(s -> s.path != null).mapToDouble(s -> s.path.getDistance()).sum());
        result.setGPXEntriesLength(gpxLength(observations));
        result.setGraph(queryGraph);
        result.setWeighting(weighting);
        return result;
    }
    //??????????????????????????????
    private List<SequenceState<State, Observation, Path>> computeViterbiSequence(List<ObservationWithCandidateStates> timeSteps) {

        StopWatch clocks = new StopWatch();
        clocks.start();
        final HmmProbabilities probabilities = new HmmProbabilities(radius, 2.0);
        final ViterbiAlgorithm<State, Observation, Path> viterbi = new ViterbiAlgorithm<>();

        int timeStepCounter = 0;
        ObservationWithCandidateStates prevTimeStep = null;
        for (ObservationWithCandidateStates timeStep : timeSteps) {
            final Map<State, Double> emissionLogProbabilities = new HashMap<>();
            Map<Transition<State>, Double> transitionLogProbabilities = new HashMap<>();
            Map<Transition<State>, Path> roadPaths = new HashMap<>();
            for (State candidate : timeStep.candidates) {
                // distance from observation to road in meters
                final double distance = candidate.getSnap().getQueryDistance();
                emissionLogProbabilities.put(candidate, probabilities.emissionLogProbability(distance));
            }

            if (prevTimeStep == null) {
                viterbi.startWithInitialObservation(timeStep.observation, timeStep.candidates, emissionLogProbabilities);
            } else {
                final double linearDistance = distanceCalc.calcDist(prevTimeStep.observation.getPoint().lat,
                        prevTimeStep.observation.getPoint().lon, timeStep.observation.getPoint().lat, timeStep.observation.getPoint().lon);

                for (State from : prevTimeStep.candidates) {
                    for (State to : timeStep.candidates) {
                        final Path path = createRouter().calcPath(from.getSnap().getClosestNode(), to.getSnap().getClosestNode(), from.isOnDirectedEdge() ? from.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE, to.isOnDirectedEdge() ? to.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);
                        if (path.isFound()) {
                            double transitionLogProbability = probabilities.transitionLogProbability(path.getDistance(), linearDistance);
                            Transition<State> transition = new Transition<>(from, to);
                            roadPaths.put(transition, path);
                            transitionLogProbabilities.put(transition, transitionLogProbability);
                        }
                    }
                }
                viterbi.nextStep(timeStep.observation, timeStep.candidates,
                        emissionLogProbabilities, transitionLogProbabilities,
                        roadPaths);
            }
            if (viterbi.isBroken()) {
                fail(timeStepCounter, prevTimeStep, timeStep);
            }

            timeStepCounter++;
            prevTimeStep = timeStep;
        }
        clocks.stop();

        return viterbi.computeMostLikelySequence();
    }

    //Ada????????????
    public MatchResult matchWithSpecificMethod(List<Observation> observations, double score, int choice){
        score_bound = score;
        int startIndex = 0, endIndex = observations.size() - 1;
        //??????
        boolean isNear = false;
        for(Observation o : observations) {
            Collection<Snap> tmp = locationIndex.
                    findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(weighting.getFlagEncoder()),radius);

            for(Snap snap : tmp) {
                if(snap.getQueryDistance() < 10) {
                    isNear = true;
                    break;
                }
            }
            if(isNear) break;
            startIndex++;
        }
        //??????
        isNear = false;
        for(; endIndex >= 0; endIndex--) {
            Observation o = observations.get(endIndex);
            Collection<Snap> tmp = locationIndex.
                    findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(weighting.getFlagEncoder()),radius);

            for(Snap snap : tmp) {
                if(snap.getQueryDistance() < 10) {
                    isNear = true;
                    break;
                }
            }
            if(isNear) break;
        }

        observations = observations.subList(startIndex, endIndex + 1);
        //?????????????????????????????????????????????
        DistanceCalc distance = new DistancePlaneProjection();
        double addi_distance = 0;
        for (int front=0; front < startIndex; front++)
            addi_distance += distance.calcDist(observations.get(front).getPoint().lat,observations.get(front).getPoint().lon, observations.get(front+1).getPoint().lat, observations.get(front+1).getPoint().lon);
        for (int back=endIndex; back < observations.size()-1; back++)
            addi_distance += distance.calcDist(observations.get(back).getPoint().lat,observations.get(back).getPoint().lon, observations.get(back+1).getPoint().lat, observations.get(back+1).getPoint().lon);
        List<Observation> filteredObservations = filterObservations(observations);//????????????????????????
        List<Collection<Snap>> splitsPerObservation;
        splitsPerObservation = filteredObservations.stream().map(o -> locationIndex.
                findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(weighting.getFlagEncoder()),radius))
                .collect(Collectors.toList());
        queryGraph = QueryGraph.create(graph, splitsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        splitsPerObservation = splitsPerObservation.stream().map(this::deduplicate).collect(Collectors.toList());
        List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, splitsPerObservation);

        //??????????????????????????????????????????id
        int observation_index = 0;
        for (ObservationWithCandidateStates timestep : timeSteps){
            timestep.id = observation_index;
            int candidate_index = 0;
            for (State candidate : timestep.candidates){
                candidate.id = candidate_index;
                candidate_index++;
            }
            observation_index++;
        }

        List<MatchResult> results = new ArrayList<>();
        List<SequenceState<State, Observation, Path>> seq = null;
        //?????????????????????
        if (choice == 0) seq = UnionMatch(timeSteps);
        else if (choice == 1) seq = AdaMatch(timeSteps);
        else if (choice == 2) seq = ReverseMatch(timeSteps);
        else if (choice == 3) seq = NewMatch(timeSteps);
        //?????????????????????

        List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.path != null).flatMap(s1 -> s1.path.calcEdges().stream()).collect(Collectors.toList());

        MatchResult result = new MatchResult(prepareEdgeMatches(seq));
        result.setMergedPath(new MapMatchedPath(queryGraph, weighting, path));
        result.setMatchMillis(seq.stream().filter(s -> s.path != null).mapToLong(s -> s.path.getTime()).sum());
        result.setMatchLength(seq.stream().filter(s -> s.path != null).mapToDouble(s -> s.path.getDistance()).sum()+addi_distance);
        result.setGPXEntriesLength(gpxLength(observations));
        result.setGraph(queryGraph);
        result.setWeighting(weighting);
        results.add(result);

        return result;
    }
    //???Ada????????????
    private List<SequenceState<State, Observation, Path>> AdaMatch(List<ObservationWithCandidateStates> TimeSteps) {
        String in_observ = "", in_candid = "", in_origin = "", in_result = "", in_path = "";//?????????????????????
        ObservationWithCandidateStates prevTimeStep = null;//?????????gps+??????????????????
        velocity.add(5.0);
        int idex = 0;
        for (ObservationWithCandidateStates TimeStep : TimeSteps) {
            idex ++;
            TimeStep.CandidateNum = TimeStep.candidates.size();
            in_observ += idex + "," + TimeStep.observation.getPoint().lat + "," + TimeStep.observation.getPoint().lon + "," + TimeStep.observation.getDirection() + '\n';
            v0 = CalcSpeedBound(velocity, TimeStep.observation.getSpeed());//????????????v0???????????????
            if (prevTimeStep != null) TimeStep.prev_delta_t = (double) (TimeStep.observation.getTimestep() - prevTimeStep.observation.getTimestep());//???????????????????????????
            TimeStep.prev = prevTimeStep;//?????????????????????
            for (State candidate : TimeStep.candidates) {
                in_origin += idex + "," + candidate.getSnap().getSnappedPoint().lat + "," + candidate.getSnap().getSnappedPoint().lon + "," + candidate.direction + '\n';
                //????????????
                PrevSpeedScore(candidate, prevTimeStep, v0, TimeStep.prev_delta_t);
                //????????????
                DirectionScore(candidate, TimeStep.observation.getDirection());
                //????????????
                PositionScore(candidate);
                //??????????????????????????????????????????
                FirstFilter(candidate, 0.01, true);
                //???????????????????????????????????????
                in_candid += idex + "," + candidate.getSnap().getSnappedPoint().lat + "," + candidate.getSnap().getSnappedPoint().lon + "," + candidate.direction + '\n';
            }
            //??????????????????????????????????????????????????????????????????
            for (State candidate : TimeStep.candidates) {
                if (!candidate.prev_delete){
                    TimeStep.delete = false;
                    break;
                }
            }
            if (TimeStep.delete) continue;
            if (PrevComputeScore(TimeStep) < score_bound) {
                TimeStep.delete = true;
                continue;//??????????????????????????????????????????????????????
            }
            //?????????????????????????????????????????????
            SecondFilter(TimeStep, 0.01, true);
            CalcMinPath(TimeStep);//??????????????????
            prevTimeStep = TimeStep;
        }
        List<SequenceState<State, Observation, Path>> result = GenerateLengthPath(TimeSteps);//????????????
        for (SequenceState<State, Observation, Path> choice : result) in_result +=  choice.state.getSnap().getSnappedPoint().lat + "," + choice.state.getSnap().getSnappedPoint().lon + "," + choice.state.direction+'\n';
        for (SequenceState<State, Observation, Path> sequence : result){
            PointList short_path = sequence.path.calcPoints();
            for (GHPoint3D point : short_path) in_path += point.lat + "," + point.lon + '\n';
        }
        WriteFile(in_observ, in_origin, in_candid, in_result, in_path,"AMM");
        return result;
    }
    //???Ada????????????
    private List<SequenceState<State, Observation, Path>> ReverseMatch(List<ObservationWithCandidateStates> TimeSteps){
        Collections.reverse(TimeSteps);
        String in_observ = "", in_candid = "", in_origin = "", in_result = "", in_path = "";//?????????????????????
        ObservationWithCandidateStates nextTimeStep = null;//?????????gps+??????????????????
        velocity.add(5.0);
        int idex = 0;
        for (ObservationWithCandidateStates TimeStep : TimeSteps) {
            idex ++;
            TimeStep.CandidateNum = TimeStep.candidates.size();
            in_observ += idex + "," + TimeStep.observation.getPoint().lat + "," + TimeStep.observation.getPoint().lon + "," + TimeStep.observation.getDirection() + '\n';
            v0 = CalcSpeedBound(velocity, TimeStep.observation.getSpeed());//????????????v0???????????????
            if (nextTimeStep != null) TimeStep.next_delta_t = (double) (nextTimeStep.observation.getTimestep() - TimeStep.observation.getTimestep());//???????????????????????????
            TimeStep.next = nextTimeStep;//?????????????????????
            for (State candidate : TimeStep.candidates) {
                in_origin += idex + "," + candidate.getSnap().getSnappedPoint().lat + "," + candidate.getSnap().getSnappedPoint().lon + "," + candidate.direction + '\n';
                RevSpeedScore(candidate, nextTimeStep, v0, TimeStep.next_delta_t);
                DirectionScore(candidate, TimeStep.observation.getDirection());
                PositionScore(candidate);
                //??????????????????????????????????????????
                FirstFilter(candidate, 0.01, false);
                //???????????????????????????????????????
                in_candid += idex + "," + candidate.getSnap().getSnappedPoint().lat + "," + candidate.getSnap().getSnappedPoint().lon + "," + candidate.direction + '\n';
            }
            for (State candidate : TimeStep.candidates){
                if (!candidate.rev_delete) {
                    TimeStep.rev_delete=false;
                    break;
                }
            }
            if (TimeStep.rev_delete) continue;
            if (RevComputeScore(TimeStep) < score_bound) {
                TimeStep.rev_delete = true;
                continue;//??????????????????????????????????????????????????????
            }
            //?????????????????????????????????????????????
            SecondFilter(TimeStep, 0.01, false);
            RevCalcMinPath(TimeStep);//??????????????????
            nextTimeStep = TimeStep;
        }
        List<SequenceState<State, Observation, Path>> result = ReverseLengthPath(TimeSteps);//????????????
        for (SequenceState<State, Observation, Path> choice : result) in_result +=  choice.state.getSnap().getSnappedPoint().lat + "," + choice.state.getSnap().getSnappedPoint().lon + "," + choice.state.direction+'\n';
        for (SequenceState<State, Observation, Path> sequence : result){
            PointList short_path = sequence.path.calcPoints();
            for (GHPoint3D point : short_path) in_path += point.lat + "," + point.lon + '\n';
        }
        WriteFile(in_observ, in_origin, in_candid, in_result, in_path,"AMM");
        return result;
    }
    //??????Ada????????????
    private List<SequenceState<State, Observation, Path>> UnionMatch(List<ObservationWithCandidateStates> TimeSteps) {
        String in_observ = "", in_candid = "", in_origin = "", in_result = "", in_path = "";//?????????????????????
        ObservationWithCandidateStates prevTimeStep = null;//?????????gps+??????????????????
        velocity.add(5.0);
        int index = 0, start_index = 0, cumulative_delete = 0, cumulative_right=0, back_index = -1;
        for (ObservationWithCandidateStates TimeStep :TimeSteps) {
            TimeStep.CandidateNum = TimeStep.candidates.size();
        }
        while (index < TimeSteps.size()) {
            ObservationWithCandidateStates TimeStep = TimeSteps.get(index);
            if (TimeStep.rev_delete){
                TimeStep.delete = true;
                cumulative_right = 0;
                index++;
                continue;
            }
            in_observ += index + "," + TimeStep.observation.getPoint().lat + "," + TimeStep.observation.getPoint().lon + "," + TimeStep.observation.getDirection() + '\n';
            v0 = CalcSpeedBound(velocity, TimeStep.observation.getSpeed());//????????????v0???????????????
            if (prevTimeStep != null) TimeStep.prev_delta_t = (double) (TimeStep.observation.getTimestep() - prevTimeStep.observation.getTimestep());//???????????????????????????
            TimeStep.prev = prevTimeStep;//?????????????????????
            for (State candidate : TimeStep.candidates) {
                in_origin += index + "," + candidate.getSnap().getSnappedPoint().lat + "," + candidate.getSnap().getSnappedPoint().lon + "," + candidate.direction + '\n';
                PrevSpeedScore(candidate, prevTimeStep, v0, TimeStep.prev_delta_t);//??????????????????
                DirectionScore(candidate, TimeStep.observation.getDirection());//????????????
                PositionScore(candidate);//????????????
                FirstFilter(candidate, 0.01, true);
                //???????????????????????????????????????
                //in_candid += index + "," + candidate.getSnap().getSnappedPoint().lat + "," + candidate.getSnap().getSnappedPoint().lon + "," + candidate.direction + '\n';
            }
            TimeStep.delete = true;
            for (State candidate : TimeStep.candidates) {
                if (!candidate.prev_delete) {
                    TimeStep.delete = false;
                    break;
                }
            }
            if (TimeStep.delete){
                //System.out.println("?????????????????????"+TimeStep.id);
                cumulative_right = 0;
                cumulative_delete++;
                index++;
                continue;//??????????????????????????????????????????????????????
            }
            int candidate_num = 0;
            for (State candidate : TimeStep.candidates) if (!candidate.prev_delete) candidate_num++;
            if (PrevComputeScore(TimeStep) < score_bound) {
                //System.out.println(PrevComputeScore(TimeStep)+"?????????????????????"+TimeStep.id);
                TimeStep.delete = true;
                cumulative_right = 0;
                index++;
                continue;//????????????????????????
            }
            //?????????????????????????????????????????????
            SecondFilter(TimeStep, 0.01, true);
            cumulative_right++;//????????????????????????
            int back_pos=-1;
            if (cumulative_delete > 1 && back_index!=index) {
                back_index = index;
                back_pos = Backward(index, TimeSteps);
                //System.out.println(index+"???????????????"+back_pos);
            }
            cumulative_delete = 0;
            if (back_pos >= 0) {
                index = back_pos;
                prevTimeStep = TimeSteps.get(index).prev;
            }
            else {
                index++;
                prevTimeStep = TimeStep;
                CalcMinPath(TimeStep);//??????????????????
            }
        }
        List<SequenceState<State, Observation, Path>> result = GenerateLengthPath(TimeSteps);//????????????
        for (SequenceState<State, Observation, Path> choice : result) in_result +=  choice.state.getSnap().getSnappedPoint().lat + "," + choice.state.getSnap().getSnappedPoint().lon + "," + choice.state.direction+'\n';
        for (SequenceState<State, Observation, Path> sequence : result){
            PointList short_path = sequence.path.calcPoints();
            for (GHPoint3D point : short_path) in_path += point.lat + "," + point.lon + '\n';
        }
        WriteFile(in_observ, in_origin, in_candid, in_result, in_path,"AMM");
        return result;
    }

    //????????????
    private List<SequenceState<State, Observation, Path>> NewMatch(List<ObservationWithCandidateStates> TimeSteps){
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        ObservationWithCandidateStates prev_timestep = null;
        for (ObservationWithCandidateStates timestep : TimeSteps){
            //??????????????????
            double weight_dist = 1;
            if (timestep.observation.accuracy > 100) weight_dist = 0;
            else if (timestep.observation.accuracy >20) weight_dist = (timestep.observation.accuracy-20.0)/(100.0-20.0);
            //?????????
            if (prev_timestep == null){
                prev_timestep = timestep;
                continue;
            }
            //??????????????????
            double weight_head = 1;
            DistanceCalc calc = new DistancePlaneProjection();
            double l = calc.calcDist(prev_timestep.observation.getPoint().lat, prev_timestep.observation.getPoint().lon, timestep.observation.getPoint().lat, timestep.observation.getPoint().lon);
            if (l < radius*1.5) weight_head = l/1.5/radius;
            double weight_con=0;//???????????????
            //??????????????????????????????????????????
            State first = null, second = null;
            double max_score=0, second_score=0;
            if (prev_timestep.chose_candidate == null){
                State prev_first = null, prev_second = null;
                for (State candidate : timestep.candidates) {
                    //????????????
                    double distance = candidate.getSnap().getQueryDistance();
                    double dist_score = Math.exp(-distance*distance/2.0/sigma_z/sigma_z)/Math.sqrt(2.0*Math.PI)/sigma_z;
                    candidate.posScore = weight_dist*dist_score;
                    //??????????????????
                    double theta = (timestep.observation.getDirection() +360.0 - candidate.direction)%360;
                    candidate.dirScore = (1.0 + Math.cos(theta/180.0*Math.PI))/2.0;
                    //??????????????????
                    double beta_gps = Math.atan((timestep.observation.getPoint().lon-prev_timestep.observation.getPoint().lon)*Math.cos(timestep.observation.getPoint().lat/180.0*Math.PI)/
                            (timestep.observation.getPoint().lat-prev_timestep.observation.getPoint().lat));
                    double head_score = (1.0+(candidate.direction+360-beta_gps)%360)/2.0;
                    candidate.rev_speed_score = weight_head * head_score;
                    //???????????????
                    double l_tr = Math.max(v0*(timestep.observation.getTimestep()-prev_timestep.observation.getTimestep()), l);
                    for (State prev_candidate : prev_timestep.candidates){
                        Path path = createRouter().calcPath(prev_candidate.getSnap().getClosestNode(), candidate.getSnap().getClosestNode(),
                                prev_candidate.isOnDirectedEdge() ? prev_candidate.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                                candidate.isOnDirectedEdge() ? candidate.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);//????????????
                        double l_p = Double.MAX_VALUE;
                        if (path.isFound()) l_p = path.getDistance();
                        double delta_l = Math.abs(l_tr-l_p);
                        double weight_con_i = 1.0;
                        if (delta_l > radius*2.0) weight_con_i = Math.exp(-delta_l);
                        double con_score = Math.exp(-delta_l);
                        if (weight_con_i > weight_con) weight_con = weight_con_i;
                        //???????????????
                        double total_score = con_score * weight_con + candidate.rev_speed_score + candidate.dirScore + candidate.posScore;
                        if (total_score > max_score){
                            second = first;
                            first = candidate; first.total_score = total_score; first.prev_speed_score = con_score;
                            prev_second = prev_first; prev_first = prev_candidate;
                            second_score = max_score; max_score = total_score;
                        }
                        else if (total_score <= max_score && total_score > second_score){
                            second = candidate;second.total_score = total_score; second.prev_speed_score = con_score;
                            prev_second = prev_candidate;
                            second_score = total_score;
                        }
                    }
                }
                //???????????????
                double K;
                if (first == null) continue;
                else if (second == null) K = 1;
                else K = (Math.abs(first.dirScore - second.dirScore)/(first.dirScore + second.dirScore) +
                            Math.abs(first.posScore - second.posScore)/(first.posScore + second.posScore) * weight_dist +
                            Math.abs(first.rev_speed_score - second.rev_speed_score)/(first.rev_speed_score + second.rev_speed_score) * weight_head +
                            Math.abs(first.prev_speed_score - second.prev_speed_score)/(first.prev_speed_score + second.prev_speed_score) * weight_con)/
                            (1.0 + weight_dist + weight_head + weight_con);
                System.out.println(K);
                if (K < 0.2) {
                    prev_timestep = timestep;
                    continue;
                }
                timestep.chose_candidate = first;
                prev_timestep.chose_candidate = prev_first;
                Path path = createRouter().calcPath(prev_timestep.chose_candidate.getSnap().getClosestNode(), timestep.chose_candidate.getSnap().getClosestNode(),
                        prev_timestep.chose_candidate.isOnDirectedEdge() ? prev_timestep.chose_candidate.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                        timestep.chose_candidate.isOnDirectedEdge() ? timestep.chose_candidate.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);
                result.add(new SequenceState<>(timestep.chose_candidate, timestep.observation, path));
                prev_timestep = timestep;
                continue;
            }
            //??????????????????
            double weight_deg;
            if (timestep.observation.getSpeed() < 3.0 || prev_timestep.observation.getSpeed() < 3.0) weight_deg = 0;
            else if (timestep.observation.getSpeed() > 6.0 && prev_timestep.observation.getSpeed() >6.0) weight_deg = 1;
            else weight_deg = (timestep.observation.getSpeed() + prev_timestep.observation.getSpeed() - 6.0)/(6.0);
            for (State candidate : timestep.candidates){
                //??????????????????
                double distance = candidate.getSnap().getQueryDistance();
                double dist_score = Math.exp(-distance*distance/2.0/sigma_z/sigma_z)/Math.sqrt(2.0*Math.PI)/sigma_z;
                candidate.posScore = weight_dist*dist_score;
                //????????????????????????
                double theta = (timestep.observation.getDirection()+360-prev_timestep.observation.getDirection())%360;
                double phi = (candidate.direction+360-prev_timestep.chose_candidate.direction)%360;
                double delta_theta = Math.min(Math.abs(theta-phi), 360-Math.abs(theta-phi));
                double deg_score = (1.0+Math.cos(delta_theta/180.0*Math.PI))/2.0;
                candidate.dirScore = weight_deg * deg_score;
                //?????????????????????????????????
                double beta_gps = Math.atan((timestep.observation.getPoint().lon-prev_timestep.observation.getPoint().lon)*Math.cos(timestep.observation.getPoint().lat/180.0*Math.PI)/(timestep.observation.getPoint().lat-prev_timestep.observation.getPoint().lat));
                double head_score = (1.0+(candidate.direction+360-beta_gps)%360)/2.0;
                candidate.rev_speed_score = weight_head * head_score;
                //??????????????????????????????
                double l_tr = Math.max(v0*(timestep.observation.getTimestep()-prev_timestep.observation.getTimestep()), l);
                Path path = createRouter().calcPath(prev_timestep.chose_candidate.getSnap().getClosestNode(), candidate.getSnap().getClosestNode(),
                        prev_timestep.chose_candidate.isOnDirectedEdge() ? prev_timestep.chose_candidate.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                        candidate.isOnDirectedEdge() ? candidate.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);//????????????
                double l_p = Double.MAX_VALUE;
                if (path.isFound()) l_p = path.getDistance();
                double delta_l = Math.abs(l_p-l_tr);
                double weight_con_i = 1.0;
                if (delta_l > radius*2.0) weight_con_i = Math.exp(-delta_l);
                candidate.prev_speed_score = Math.exp(-delta_l);
                if (weight_con_i > weight_con) weight_con = weight_con_i;
                //???????????????
                candidate.total_score = candidate.prev_speed_score + candidate.rev_speed_score + candidate.dirScore + candidate.posScore;
                if (candidate.total_score> max_score){
                    second_score = max_score;
                    max_score = candidate.total_score;
                    second = first;
                    first = candidate;
                }
                else if (candidate.total_score<=max_score && candidate.total_score>second_score){
                    second_score = candidate.total_score;
                    second = candidate;
                }
            }
            //?????????????????????
            double K;
            if (first == null) continue;
            else if (second == null) K = 1;
            else K = (Math.abs(first.dirScore - second.dirScore)/(first.dirScore + second.dirScore) * weight_deg +
                        Math.abs(first.posScore - second.posScore)/(first.posScore + second.posScore) * weight_dist +
                        Math.abs(first.rev_speed_score - second.rev_speed_score)/(first.rev_speed_score + second.rev_speed_score) * weight_head +
                        Math.abs(first.prev_speed_score - second.prev_speed_score)/(first.prev_speed_score + second.prev_speed_score) * weight_con)/
                        (weight_deg + weight_dist + weight_head + weight_con);
            System.out.println(K);
            if (K < 0.01) continue;
            timestep.chose_candidate = first;
            Path path = createRouter().calcPath(prev_timestep.chose_candidate.getSnap().getClosestNode(), timestep.chose_candidate.getSnap().getClosestNode(),
                    prev_timestep.chose_candidate.isOnDirectedEdge() ? prev_timestep.chose_candidate.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                    timestep.chose_candidate.isOnDirectedEdge() ? timestep.chose_candidate.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);
            result.add(new SequenceState<>(timestep.chose_candidate, timestep.observation, path));
            prev_timestep = timestep;
        }
        return result;
    }

    private void fail(int timeStepCounter, ObservationWithCandidateStates prevTimeStep, ObservationWithCandidateStates timeStep) {
        String likelyReasonStr = "";
        if (prevTimeStep != null) {
            double dist = distanceCalc.calcDist(prevTimeStep.observation.getPoint().lat, prevTimeStep.observation.getPoint().lon, timeStep.observation.getPoint().lat, timeStep.observation.getPoint().lon);
            if (dist > 2000) {
                likelyReasonStr = "Too long distance to previous measurement? "
                        + Math.round(dist) + "m, ";
            }
        }

        throw new IllegalArgumentException("Sequence is broken for submitted track at time step "
                + timeStepCounter + ". "
                + likelyReasonStr + "observation:" + timeStep.observation + ", "
                + timeStep.candidates.size() + " candidates: "
                + getSnappedCandidates(timeStep.candidates)
                + ". If a match is expected consider increasing max_visited_nodes.");
    }

    //????????????????????????
    private BidirRoutingAlgorithm createRouter() {
        BidirRoutingAlgorithm router;
        if (landmarks != null) {
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
    private List<EdgeMatch> prepareEdgeMatches(List<SequenceState<State, Observation, Path>> seq) {
        // This creates a list of directed edges (EdgeIteratorState instances turned the right way),
        // each associated with 0 or more of the observations.
        // These directed edges are edges of the real street graph, where nodes are intersections.
        // So in _this_ representation, the path that you get when you just look at the edges goes from
        // an intersection to an intersection.

        // Implementation note: We have to look at both states _and_ transitions, since we can have e.g. just one state,
        // or two states with a transition that is an empty path (observations snapped to the same node in the query graph),
        // but these states still happen on an edge, and for this representation, we want to have that edge.
        // (Whereas in the ResponsePath representation, we would just see an empty path.)

        // Note that the result can be empty, even when the input is not. Observations can be on nodes as well as on
        // edges, and when all observations are on the same node, we get no edge at all.
        // But apart from that corner case, all observations that go in here are also in the result.

        // (Consider totally forbidding candidate states to be snapped to a point, and make them all be on directed
        // edges, then that corner case goes away.)
        List<EdgeMatch> edgeMatches = new ArrayList<>();
        List<State> states = new ArrayList<>();
        EdgeIteratorState currentDirectedRealEdge = null;
        for (SequenceState<State, Observation, Path> transitionAndState : seq) {
            // transition (except before the first state)
            if (transitionAndState.path != null) {
                for (EdgeIteratorState edge : transitionAndState.path.calcEdges()) {
                    EdgeIteratorState newDirectedRealEdge = resolveToRealEdge(edge);
                    if (currentDirectedRealEdge != null) {
                        if (!equalEdges(currentDirectedRealEdge, newDirectedRealEdge)) {
                            EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
                            edgeMatches.add(edgeMatch);
                            states = new ArrayList<>();
                        }
                    }
                    currentDirectedRealEdge = newDirectedRealEdge;
                }
            }
            // state
            if (transitionAndState.state.isOnDirectedEdge()) { // as opposed to on a node
                EdgeIteratorState newDirectedRealEdge = resolveToRealEdge(transitionAndState.state.getOutgoingVirtualEdge());
                if (currentDirectedRealEdge != null) {
                    if (!equalEdges(currentDirectedRealEdge, newDirectedRealEdge)) {
                        EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
                        edgeMatches.add(edgeMatch);
                        states = new ArrayList<>();
                    }
                }
                currentDirectedRealEdge = newDirectedRealEdge;
            }
            states.add(transitionAndState.state);
        }
        if (currentDirectedRealEdge != null) {
            EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
            edgeMatches.add(edgeMatch);
        }
        return edgeMatches;
    }
    public double gpxLength(List<Observation> gpxList) {
        if (gpxList.isEmpty()) {
            return 0;
        } else {
            double gpxLength = 0;
            Observation prevEntry = gpxList.get(0);
            for (int i = 1; i < gpxList.size(); i++) {
                Observation entry = gpxList.get(i);
                gpxLength += distanceCalc.calcDist(prevEntry.getPoint().lat, prevEntry.getPoint().lon, entry.getPoint().lat, entry.getPoint().lon);
                prevEntry = entry;
            }
            return gpxLength;
        }
    }
    private boolean equalEdges(EdgeIteratorState edge1, EdgeIteratorState edge2) {
        return edge1.getEdge() == edge2.getEdge()
                && edge1.getBaseNode() == edge2.getBaseNode()
                && edge1.getAdjNode() == edge2.getAdjNode();
    }
    private EdgeIteratorState resolveToRealEdge(EdgeIteratorState edgeIteratorState) {
        if (queryGraph.isVirtualNode(edgeIteratorState.getBaseNode()) || queryGraph.isVirtualNode(edgeIteratorState.getAdjNode())) {
            return graph.getEdgeIteratorStateForKey(((VirtualEdgeIteratorState) edgeIteratorState).getOriginalEdgeKey());
        } else {
            return edgeIteratorState;
        }
    }
    private String getSnappedCandidates(Collection<State> candidates) {
        String str = "";
        for (State gpxe : candidates) {
            if (!str.isEmpty()) {
                str += ", ";
            }
            str += "distance: " + gpxe.getSnap().getQueryDistance() + " to "
                    + gpxe.getSnap().getSnappedPoint();
        }
        return "[" + str + "]";
    }
    private static class MapMatchedPath extends Path {
        MapMatchedPath(Graph graph, Weighting weighting, List<EdgeIteratorState> edges) {
            super(graph);
            int prevEdge = EdgeIterator.NO_EDGE;
            for (EdgeIteratorState edge : edges) {
                addDistance(edge.getDistance());
                addTime(GHUtility.calcMillisWithTurnMillis(weighting, edge, false, prevEdge));
                addEdge(edge.getEdge());
                prevEdge = edge.getEdge();
            }
            if (edges.isEmpty()) {
                setFound(false);
            } else {
                setFromNode(edges.get(0).getBaseNode());
                setFound(true);
            }
        }
    }

    //????????????????????????
    private void WriteFile(String in_observ, String in_origin, String in_candid, String in_result, String in_path, String method){
        in_observ = "num,la,lon,dir\n" + in_observ;
        in_origin = "num,la,lon,dir\n" + in_origin;
        in_candid = "num,la,lon,dir\n" + in_candid;
        in_result = "la,lon,dir\n" + in_result;
        in_path = "la,lon\n" + in_path;
        File obser = new File("map-data/result/"+method+"/observation.txt");
        File origi = new File("map-data/result/"+method+"/origin.txt");
        File candi = new File("map-data/result/"+method+"/candidate.txt");
        File resul = new File("map-data/result/"+method+"/result.txt");
        File pathh = new File("map-data/result/"+method+"/path.txt");
        try {
            FileOutputStream fos1 = new FileOutputStream(obser);OutputStreamWriter dos1 = new OutputStreamWriter(fos1);dos1.write(in_observ);dos1.close();
            FileOutputStream fos2 = new FileOutputStream(origi);OutputStreamWriter dos2 = new OutputStreamWriter(fos2);dos2.write(in_origin);dos2.close();
            FileOutputStream fos3 = new FileOutputStream(candi);OutputStreamWriter dos3 = new OutputStreamWriter(fos3);dos3.write(in_candid);dos3.close();
            FileOutputStream fos4 = new FileOutputStream(resul);OutputStreamWriter dos4 = new OutputStreamWriter(fos4);dos4.write(in_result);dos4.close();
            FileOutputStream fos5 = new FileOutputStream(pathh);OutputStreamWriter dos5 = new OutputStreamWriter(fos5);dos5.write(in_path);dos5.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    //??????????????????v0
    private double CalcSpeedBound(List<Double> V, double v){
        if (V.size() < 5 && v < 50.0){
            V.add(Math.max(v,5.0));
        }
        else if(V.size() >= 5 && v < 50.0){
            V.remove(0);
            V.add(Math.max(v,5.0));
        }
        double tmp_sum = 0.0;
        for (double speed : V) tmp_sum += speed;
        return tmp_sum/V.size();//????????????????????????????????????????????????
    }

    //??????????????????????????????????????????????????????
    private void PrevSpeedScore(State to, ObservationWithCandidateStates prev, double v0, double t0){
        if (prev == null) to.prev_speed_score = Utils.getAdjustSpeedProb(v0, v0);
        else{
            double[] score_from_to = new double[prev.CandidateNum];
            double[] velocity_from_to = new double[prev.CandidateNum];
            double s_v = 0;
            for (State from : prev.candidates){
                if (from.prev_delete) continue;
                double dist;
                final Path path = createRouter().calcPath(from.getSnap().getClosestNode(), to.getSnap().getClosestNode(),
                        from.isOnDirectedEdge() ? from.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                        to.isOnDirectedEdge() ? to.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);//????????????
                //????????????from-to???????????????????????????
                if (path.isFound()) {
                    dist = path.getDistance();
                    velocity_from_to[from.id] = dist / t0;
                    score_from_to[from.id] = Utils.getAdjustSpeedProb(dist / t0, v0);
                } else {
                    velocity_from_to[from.id] = Double.MAX_VALUE;
                    score_from_to[from.id] = 0.0;
                }
                s_v += score_from_to[from.id] * from.prev_prob;
            }
            //????????????????????????????????????????????????????????????
            to.PrevTransitionScore.clear();
            to.PrevTransitionVelocity.clear();
            for (int i=0; i<prev.CandidateNum; i++) {
                to.PrevTransitionVelocity.add(velocity_from_to[i]);
                to.PrevTransitionScore.add(score_from_to[i]);
            }
            to.prev_speed_score = s_v;
        }
    }
    private void RevSpeedScore(State candidate, ObservationWithCandidateStates next, double v0, double t0) {
        if (next == null) candidate.rev_speed_score = Utils.getAdjustSpeedProb(v0, v0);
        else {
            double[] score_from_to = new double[next.CandidateNum];
            double[] velocity_from_to = new double[next.CandidateNum];
            double s_v = 0;
            for (State to : next.candidates) {
                double dist;
                final Path path = createRouter().calcPath(candidate.getSnap().getClosestNode(), to.getSnap().getClosestNode(),
                        candidate.isOnDirectedEdge() ? candidate.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                        to.isOnDirectedEdge() ? to.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);//????????????
                //????????????from-to???????????????????????????
                if (path.isFound()) {
                    dist = path.getDistance();
                    velocity_from_to[to.id] = dist / t0;
                    score_from_to[to.id] = Utils.getAdjustSpeedProb(dist / t0, v0);
                } else {
                    velocity_from_to[to.id] = Double.MAX_VALUE;
                    score_from_to[to.id] = 0.0;
                }
                s_v += score_from_to[to.id] * to.rev_prob;
            }
            //????????????????????????????????????????????????????????????
            candidate.RevTransitionScore.clear();
            candidate.RevTransitionVelocity.clear();
            for (int i = 0; i < next.CandidateNum; i++) {
                candidate.RevTransitionVelocity.add(velocity_from_to[i]);
                candidate.RevTransitionScore.add(score_from_to[i]);
            }
            candidate.rev_speed_score = s_v;
        }
    }
    private void DirectionScore(State to, double Direction){
        PointList pointList = null;
        double pathDirection = 0.0;
        double preLat,postLat, postLon, preLon;
        if(to.isOnDirectedEdge()) pointList = to.getIncomingVirtualEdge().fetchWayGeometry(FetchMode.ALL);
        else{
            EdgeIteratorState tmp = to.getSnap().getClosestEdge();
            pointList = tmp.fetchWayGeometry(FetchMode.ALL);
        }
        if (pointList.size() >= 2) {
            int index = to.isOnDirectedEdge() ? 0 :  to.getSnap().getWayIndex();
            if(index == 0) {
                preLat = pointList.getLat(index);
                preLon = pointList.getLon(index);
                postLat = pointList.getLat(index+1);
                postLon = pointList.getLon(index+1);
            }
            else {
                preLat = pointList.getLat(index-1);
                preLon = pointList.getLon(index-1);
                postLat = pointList.getLat(index);
                postLon = pointList.getLon(index);
            }
            pathDirection = Utils.calDirection(preLat, preLon, postLat, postLon);
            if(to.isOnDirectedEdge()) {
                boolean test = to.getIncomingVirtualEdge().getReverse(EdgeIteratorState.REVERSE_STATE);
                if(!test) pathDirection = Utils.calDirection(postLat, postLon, preLat, preLon);
            }
        }
        else pathDirection = Double.MAX_VALUE;//????????????????????????????????????????????????
        to.direction = pathDirection;
        double delta_theta1 = Math.min(Math.abs(pathDirection - Direction),360.0-Math.abs(pathDirection - Direction));
        to.dirScore = Math.sqrt(2.0) * Distributions.standardNormalDistribution(delta_theta1/sigma_theta);
    };
    private void PositionScore(State to){
        double distance = to.getSnap().getQueryDistance();
        to.posScore = Math.sqrt(2.0) * Distributions.standardNormalDistribution(Math.abs(distance)/sigma_z);
    };

    //???????????????
    private void FirstFilter(State to, double Bound, boolean direction) {
        if (direction) {
            to.prev_delete = to.prev_speed_score < Bound;
        }
        else to.rev_delete = to.rev_speed_score < Bound;
    }
    private void SecondFilter(ObservationWithCandidateStates TimeStep, double Bound, boolean direction){
        if (direction) {
            double sumP = 0.0;
            for (State candidate : TimeStep.candidates) {
                if (!candidate.prev_delete){
                    if (candidate.prev_prob < Bound) candidate.prev_delete = true;
                else sumP += candidate.prev_prob;
                }
            }
            for (State candidate : TimeStep.candidates) if (!candidate.prev_delete) candidate.prev_prob = candidate.prev_prob / sumP;//???????????????
        }
        else {
            double sumP = 0.0;
            for (State candidate : TimeStep.candidates) {
                if (!candidate.rev_delete){
                    if (candidate.rev_prob < Bound) candidate.rev_delete = true;
                    else sumP += candidate.rev_prob;
                }
            }
            for (State candidate : TimeStep.candidates) if (!candidate.rev_delete) candidate.rev_prob = candidate.rev_prob / sumP;//???????????????
        }
    }

    //????????????
    private double PrevComputeScore(ObservationWithCandidateStates TimeStep){
        int candidate_num = 0;
        for (State candidate : TimeStep.candidates) if (!candidate.prev_delete) candidate_num++;
        double[][] ScoreMatrix = new double[3][candidate_num];
        int candidateCount = 0;
        for (State candidate : TimeStep.candidates) {
            if (!candidate.prev_delete){
                ScoreMatrix[1][candidateCount] = candidate.prev_speed_score;
                ScoreMatrix[2][candidateCount] = candidate.dirScore;
                ScoreMatrix[0][candidateCount] = candidate.posScore;
                candidateCount++;
            }
        }
        /*for (int i=0 ;i < 3; i++){
            for (int j=0;j < candidateCount; j++){
                System.out.print(ScoreMatrix[i][j] + " ");
            }
            System.out.println("?????????");
        }*/
        //??????????????????
        RealVector confidence = Compute.getResult(ScoreMatrix, candidate_num, 3);
        double[] confindenceArr = confidence.toArray();
        RealVector weightVector =  confidence.getSubVector(0, 3);
        double score = 0.0;
        RealMatrix S = MatrixUtils.createRealMatrix(ScoreMatrix);
        for(int i = 0; i < S.getColumnDimension(); i++) {
            score += weightVector.dotProduct(S.getColumnVector(i)) * confindenceArr[3 + i];
        }
        candidateCount = 0;
        for (State candidate : TimeStep.candidates) {
            if (!candidate.prev_delete){
                candidate.prev_prob = confindenceArr[3 + candidateCount];//??????????????????
                candidateCount++;
            }
        }
        return score;
    }
    private double RevComputeScore(ObservationWithCandidateStates TimeStep){
        int candidate_num = 0;
        for (State candidate : TimeStep.candidates) if (!candidate.rev_delete) candidate_num++;
        double[][] ScoreMatrix = new double[3][candidate_num];
        int candidateCount = 0;
        for (State candidate : TimeStep.candidates) {
            if (!candidate.rev_delete){
                ScoreMatrix[1][candidateCount] = candidate.rev_speed_score;
                ScoreMatrix[2][candidateCount] = candidate.dirScore;
                ScoreMatrix[0][candidateCount] = candidate.posScore;
                candidateCount++;
            }
        }
        RealVector confidence = Compute.getResult(ScoreMatrix, candidate_num, 3);
        double[] confindenceArr = confidence.toArray();
        RealVector weightVector =  confidence.getSubVector(0, 3);
        double score = 0.0;
        RealMatrix S = MatrixUtils.createRealMatrix(ScoreMatrix);
        for(int i = 0; i < S.getColumnDimension(); i++) {
            score += weightVector.dotProduct(S.getColumnVector(i)) * confindenceArr[3 + i];
        }
        candidateCount = 0;
        for (State candidate : TimeStep.candidates) {
            if (!candidate.rev_delete){
                candidate.rev_prob = confindenceArr[3 + candidateCount];//??????????????????
                candidateCount++;
            }
        }
        return score;
    }

    //????????????
    private int Backward(int begin_index, List<ObservationWithCandidateStates> TimeSteps){
        int count=0;
        List<ObservationWithCandidateStates> Tmp = TimeSteps.subList(0, begin_index+1);
        Collections.reverse(Tmp);
        int simultaneous_right = 0 ,end_index = 0;
        ObservationWithCandidateStates nexttmp = null;
        int i = 0,origin_count = 0;
        while (i<Tmp.size() && simultaneous_right <= 1){
            ObservationWithCandidateStates tmp = Tmp.get(i);
            if (tmp.delete) origin_count++;
            if (nexttmp != null)
                tmp.next_delta_t = (double) (nexttmp.observation.getTimestep() - tmp.observation.getTimestep());//???????????????????????????
            tmp.next = nexttmp;//?????????????????????
            for (State tmp_candid : tmp.candidates) {
                RevSpeedScore(tmp_candid, nexttmp, v0, tmp.next_delta_t);//????????????
                DirectionScore(tmp_candid, tmp.observation.getDirection());//????????????
                PositionScore(tmp_candid);//????????????
                FirstFilter(tmp_candid, 0.01, false);
            }
            tmp.rev_delete = true;
            for (State tmp_candid : tmp.candidates)
                if (!tmp_candid.rev_delete) {
                    tmp.rev_delete = false;//??????????????????????????????????????????????????????
                    break;
                }
            if (tmp.rev_delete) {
                i++;
                simultaneous_right = 0;
                count++;
                continue;
            }
            if (RevComputeScore(tmp) < score_bound) {
                i++;
                simultaneous_right = 0;//??????????????????????????????????????????????????????
                continue;
            }
            nexttmp = tmp;
            if (tmp.rev_delete == tmp.delete && !tmp.rev_delete) simultaneous_right++;
            end_index = tmp.id;
            i++;
        }
        Collections.reverse(Tmp);
        if (count < origin_count) return end_index;
        else {
            for (ObservationWithCandidateStates tm : TimeSteps.subList(end_index,begin_index+1)) tm.rev_delete = false;
            return -1;
        }
    }

    //????????????????????????????????????????????????
    private void CalcMinPath(ObservationWithCandidateStates TimeStep) {
        double t = TimeStep.prev_delta_t;
        if (TimeStep.prev != null){
            //System.out.println(TimeStep.prev.id);
            for (State candidate : TimeStep.candidates){
                double min_length = Double.MAX_VALUE;
                for (State prev_candidate : TimeStep.prev.candidates){
                    double dist = Double.MAX_VALUE;
                    if (!prev_candidate.prev_delete) dist = candidate.PrevTransitionVelocity.get(prev_candidate.id) * t + prev_candidate.prev_min_path;
                    //if (TimeStep.id == 27) System.out.println(candidate.PrevTransitionVelocity.get(prev_candidate.id));
                    if (dist < min_length){
                        candidate.prev_min_path = dist;
                        candidate.max_prev = prev_candidate;
                        min_length = dist;
                    }

                }
            }
        }
        else {
            for (State candidate : TimeStep.candidates) candidate.prev_min_path = 0;
        }
        double min_path = Double.MAX_VALUE;
        for (State candidate : TimeStep.candidates) if (candidate.prev_min_path<min_path && !candidate.prev_delete) min_path = candidate.prev_min_path;
        //System.out.println(TimeStep.id + "??????"  + min_path);

    }
    private List<SequenceState<State, Observation, Path>> GenerateLengthPath(List<ObservationWithCandidateStates> TimeSteps){
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        State last = null;//??????????????????
        int last_num = TimeSteps.size()-1;
        //?????????
        while(last == null || last.max_prev == null) {
            ObservationWithCandidateStates timestep = TimeSteps.get(last_num);
            double min_dist = Double.MAX_VALUE;
            for (State candidate : timestep.candidates){
                if (candidate.prev_min_path<min_dist && candidate.max_prev != null) {
                    last = candidate;
                    min_dist = candidate.prev_min_path;
                }
            }
            last_num--;
            if(last_num <= 0) {
                System.out.println("??????????????????????????????????????????????????????");
                break;
            };
        }
        //??????????????????
        while(last != null && last.max_prev != null) {
            Path path = createRouter().calcPath(last.max_prev.getSnap().getClosestNode(), last.getSnap().getClosestNode(),
                    last.max_prev.isOnDirectedEdge() ? last.max_prev.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                    last.isOnDirectedEdge() ? last.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);
            result.add(new SequenceState<State, Observation, Path>(last, last.getEntry(), path));
            last = last.max_prev;
            PointList tmp_list = path.calcPoints();
            tmp_list.reverse();
        }
        //System.out.println("????????????"+ last.getEntry().toString());
        Collections.reverse(result);
        return result;
    }

    private void RevCalcMinPath(ObservationWithCandidateStates TimeStep) {
        double t = TimeStep.next_delta_t;
        if (TimeStep.next != null){
            for (State candidate : TimeStep.candidates){
                double min_length = Double.MAX_VALUE;
                for (State next_candidate : TimeStep.next.candidates){
                    double dist = candidate.RevTransitionVelocity.get(next_candidate.id) * t + next_candidate.rev_min_path;
                    if (dist < min_length){
                        candidate.rev_min_path = dist;
                        candidate.max_next = next_candidate;
                        min_length = dist;
                    }
                }
            }
        }
        else {
            for (State candidate : TimeStep.candidates) candidate.rev_min_path = 0;
        }
    }
    private List<SequenceState<State, Observation, Path>> ReverseLengthPath(List<ObservationWithCandidateStates> TimeSteps){
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        State first = null;//??????????????????
        int first_num = TimeSteps.size()-1;
        //?????????
        while(first == null || first.max_next == null) {
            ObservationWithCandidateStates timestep = TimeSteps.get(first_num);
            double min_dist = Double.MAX_VALUE;
            for (State candidate : timestep.candidates){
                if (candidate.rev_min_path < min_dist && candidate.max_next != null) {
                    first = candidate;
                    min_dist = candidate.rev_min_path;
                }
            }
            first_num--;
            if(first_num <= 0) {
                System.out.println("??????????????????????????????????????????????????????");
                break;
            };
        }
        //??????????????????
        while(first != null && first.max_next != null) {
            Path path = createRouter().calcPath(first.getSnap().getClosestNode(), first.max_next.getSnap().getClosestNode(),
                    first.isOnDirectedEdge() ? first.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE,
                    first.max_next.isOnDirectedEdge() ? first.max_next.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);
            result.add(new SequenceState<State, Observation, Path>(first, first.getEntry(), path));
            first = first.max_next;
        }
        return result;
    }
}