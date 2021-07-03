/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.matching;

import com.graphhopper.util.shapes.GHPoint;

import java.util.Objects;

public class Observation {
    private GHPoint point;

    private long timeStep;
    private double lat;
    private double lon;
    public double accuracy;
    private double speed;
    private double direction;
    public double avgSpeed;

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getDirection() {
        return direction;
    }

    public long getTimestep() {
        return timeStep;
    }

    public Observation(long timeStep, double lat, double lon) {
        this.timeStep = timeStep;
        this.point = new GHPoint(lat, lon);
    }

    public Observation(double lat, double lon){
        this.point = new GHPoint(lat, lon);
    }

    public Observation(long timeStep, double lat, double lon, double acc) {
        this.timeStep = timeStep;
        this.point = new GHPoint(lat, lon);
        this.accuracy = acc;
    }

    public Observation(long timeStep, double lat, double lon,  double speed,
                       double direction, double acc) {
        this.timeStep = timeStep;
        this.point = new GHPoint(lat, lon);
        this.accuracy = acc;
        this.speed = speed;
        this.direction = direction;
    }

    public Observation(GHPoint p) {
        this.point = p;
    }

    public GHPoint getPoint() {
        return point;
    }

    @Override
    public String toString() {
        return "Observation{" +
                "point=" + point +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Observation that = (Observation) o;
        return Objects.equals(point, that.point);
    }

    @Override
    public int hashCode() {
        return Objects.hash(point);
    }
}
