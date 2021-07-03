/**
 * Copyright (C) 2015-2016, BMW Car IT GmbH and BMW AG
 * Author: Stefan Holder (stefan.holder@bmw.de)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.graphhopper.matching;

import java.util.ArrayList;
import java.util.Collection;

public class ObservationWithCandidateStates {

    public int id;
    public boolean transform = false;//是否用反向结果代替正向结果
    public boolean rev_delete = false;
    public boolean delete = true;
    public ObservationWithCandidateStates prev;
    public ObservationWithCandidateStates next;
    public int CandidateNum;
    public double prev_delta_t = 1;
    public double next_delta_t = 1;
    public State chose_candidate;
    /**
     * Observation made at this time step.
     */
    public Observation observation;

    /**
     * State candidates at this time step.
     */
    public Collection<State> candidates;

    public ObservationWithCandidateStates(Observation observation, Collection<State> candidates) {
        if (observation == null || candidates == null) {
            throw new NullPointerException("observation and candidates must not be null.");
        }
        this.observation = observation;
        this.candidates = candidates;
    }
}
