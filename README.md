## AMM

The latest version is available at https://github.com/Hanwen-Hu/AMM.

### Usage

The Map Matching algorithm is running on [Graphhopper](https://github.com/graphhopper/map-matching).


Java 8 and Maven >=3.3 are required.

Build:

```bash
mvn package -DskipTests
```

Then you need to import an OSM map for the area you want to do map-matching on Shanghai.

```bash
java -jar matching-web/target/graphhopper-map-matching-web-3.0-SNAPSHOT.jar import map-data/shanghai.osm.pbf
```



#### Java usage

Have a look at `MapMatchingResource.java` to see how the web service is implemented on top
of library functions to get an idea how to use map matching in your own project.

Use this Maven dependency:
```xml
<dependency>
    <groupId>com.graphhopper</groupId>
    <artifactId>graphhopper-map-matching-core</artifactId>
    <version>3.0-SNAPSHOT</version>
</dependency>
```

### Note

Note that the edge and node IDs from GraphHopper will change for different PBF files,
like when updating the OSM data.

### About

The map matching algorithm mainly follows the approach described in

*Newson, Paul, and John Krumm. "Hidden Markov map matching through noise and sparseness."
Proceedings of the 17th ACM SIGSPATIAL International Conference on Advances in Geographic
Information Systems. ACM, 2009.*

This algorithm works as follows. For each input GPS position, a number of
map matching candidates within a certain radius around the GPS position is computed.
The [Viterbi algorithm](https://en.wikipedia.org/wiki/Viterbi_algorithm) as provided by the
[hmm-lib](https://github.com/bmwcarit/hmm-lib) is then used to compute the most likely sequence
of map matching candidates. Thereby, the distances between GPS positions and map matching
candidates as well as the routing distances between consecutive map matching candidates are taken
into account. The GraphHopper routing engine is used to find candidates and to compute routing
distances.

Before GraphHopper 0.8, [this faster but more heuristic approach](https://karussell.wordpress.com/2014/07/28/digitalizing-gpx-points-or-how-to-track-vehicles-with-graphhopper/)
was used.
