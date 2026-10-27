#!/usr/bin/env python3

import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment

#Update(), at the bottom of the file is the only function called from the outside

#Each detected object will have its own track
class ObjectTrack:
    def __init__(self, track_id, observation, timestamp, tracker):
        self.id = track_id
        self.obj_class = observation['class_id']
        self.observations = 1
        self.last_seen = float(timestamp)
        self.last_predicted = float(timestamp)
        self.min_observations = tracker.min_observations
        self.process_std_per_sqrt_second = tracker.process_std_per_sqrt_second
        self.existence_score = 1.0  #Heuristic, will add +1 and -1
        self.confirmation_score = tracker.confirmation_score
        self.confirmed = False
        self.expected_misses = 0
        self.last_visibility_check = float(timestamp)

        #Kalman Parameters
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.asarray(observation['position'], dtype=float).reshape(3, 1)
        self.kf.F = np.eye(3)
        self.kf.H = np.eye(3)
        self.kf.P = np.eye(3) * tracker.initial_std**2
        self.kf.Q = np.zeros((3, 3))
        self.kf.R = np.asarray(observation['covariance'], dtype=float)

    @property
    def status(self):
        return 'confirmed' if self.confirmed else 'tentative'

    @property
    def position(self):
        return self.kf.x.reshape(3).copy()

    @property
    def covariance(self):
        return self.kf.P.copy()

    #Predict where we think object will be and save the time of the prediction
    def predict(self, timestamp):
        dt = float(timestamp) - self.last_predicted
        self.kf.Q = np.eye(3) * self.process_std_per_sqrt_second**2 * dt #Kalman equation
        self.kf.predict()
        self.last_predicted = float(timestamp)

    def update(self, observation, timestamp):

        #Send object XYZ, and covariance matrix to kalman update function
        self.kf.update(
            np.asarray(observation['position'], dtype=float).reshape(3, 1),
            R=np.asarray(observation['covariance'], dtype=float),
        )

        #Update variables used to determine if object exists
        self.observations += 1
        self.last_seen = float(timestamp)
        self.last_visibility_check = float(timestamp)
        self.expected_misses = 0
        self.existence_score = min(12.0, self.existence_score + 1.0)

        if (self.observations >= self.min_observations and self.existence_score >= self.confirmation_score):
            self.confirmed = True

    #Called when we SHOULD have seen something but didn't
    def missed_expected_detection(self, timestamp):
        self.last_visibility_check = float(timestamp) #Used to determine if this function should be called
        self.expected_misses += 1
        self.existence_score = max(-12.0, self.existence_score - 1.0)

#This is for all the objects
class ObjectTracker:

    #Set Parameters
    #TODO, motivate/verify
    def __init__(
        self,
        min_observations=5,
        initial_std=0.5,
        measurement_std=0.20,
        process_std_per_sqrt_second=0.01,
        chi2_gate_3d=11.345,   # 99% chi-square, 3 degrees of freedom
        max_physical_distance=2.0,
        confirmation_score=4.0,
        max_expected_misses=4,
        min_miss_interval=1.0,
        min_visible_distance=0.8,
        max_visible_distance=6.0,
        frame_margin=0.12,
    ):
        self.tracks = []
        self.next_id = 1
        self.min_observations = int(min_observations)
        self.initial_std = float(initial_std)
        self.measurement_std = float(measurement_std)
        self.process_std_per_sqrt_second = float(process_std_per_sqrt_second)
        self.chi2_gate_3d = float(chi2_gate_3d)
        self.max_physical_distance = float(max_physical_distance)
        self.confirmation_score = float(confirmation_score)
        self.max_expected_misses = int(max_expected_misses)
        self.min_miss_interval = float(min_miss_interval)
        self.min_visible_distance = float(min_visible_distance)
        self.max_visible_distance = float(max_visible_distance)
        self.frame_margin = float(frame_margin)
        self.new_track_cost = 7.0
        self.missed_track_cost = 7.0

    def confirmed_tracks(self):
        return [track for track in self.tracks if track.confirmed]

    #TODO, see if this can be removed
    def _measurement_covariance(self, observation):
        #If we have a covariance matrix use it, otherwise use the default one
        if 'covariance' in observation:
            matrix = np.asarray(observation['covariance'], dtype=float)
        else:
            matrix = np.eye(3) * self.measurement_std**2

        #Error handling for matrix dimensions
        if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            raise ValueError('Observation covariance must be finite 3x3')

        #making matrix symmetrical (basically *2/2), and adding tiny variance on diagonal
        #to lower chance of singularity, which will cause error later
        return (matrix + matrix.T) / 2 + np.eye(3) * 1e-8

    #Make a list out of valid position observations
    def _normalise_observations(self, observations):
        result = []
        for obs in observations:
            p = np.asarray(obs['position'], dtype=float).reshape(3)
            if not np.isfinite(p).all():
                continue
            result.append({**obs, 'position': p,
                           'covariance': self._measurement_covariance(obs)})
        return result

    #Given the results from the project function,
    #Would an object be visible, i.e on the screen and close enough to be seen?
    def _visible(self, projection):
        if projection is None:
            return False

        depth = projection['forward_depth']
        u, v = projection['uv']
        width, height = projection['image_size']
        margin = self.frame_margin

        return (self.min_visible_distance <= depth <= self.max_visible_distance
                and margin * width <= u <= (1 - margin) * width
                and margin * height <= v <= (1 - margin) * height)

    #Compare track with observed object and see how well they match
    def _candidate_cost(self, track, obs, projected):

        #Wrong class
        if obs['class_id'] != track.obj_class:
            return None

        #Is the eucledian distance too long?
        delta = obs['position'] - track.position
        distance = float(np.linalg.norm(delta))
        if distance > self.max_physical_distance:
            return None

        #Mahalanobis squared is d2
        S = track.covariance + obs['covariance']
        try:
            d2 = float(delta @ np.linalg.solve(S, delta))
        except np.linalg.LinAlgError:
            return None
        if d2 > self.chi2_gate_3d:
            return None
        cost = d2

        #Pixel error in tracks projected object and real object observation
        if projected is not None and 'bbox_center' in obs:
            pixel_error = np.linalg.norm(
                np.asarray(projected['uv']) - np.asarray(obs['bbox_center'])
            )
            #This is a heuristic to match the size of the mahalanobis cost with the pixel distance cost
            #TODO, change/test/verify/remove/motivate
            cost += min(2.0, (pixel_error / 45.0) ** 2)

        #Making existing tracks more likely to match
        if track.confirmed:
            cost -= 1.0

        return cost

    #Read
    def update(self, observations, timestamp, project=None, raw_detections=None):

        observations = self._normalise_observations(observations)   #Remove invalid observations
        raw_detections = raw_detections or []
        timestamp = float(timestamp)

        #Predict position of each tracks object
        for track in self.tracks:
            track.predict(timestamp)

        #Make a projection for each tracks object onto a 2D screen
        projections = []
        for track in self.tracks:
            projections.append(project(track.position) if project is not None else None)

        #Loop through all observation and track combinations
        #Create matrix that contain a cost for each for how well they match
        #Find list of optimal pairings, and do kalman update on them
        n_tracks, n_obs = len(self.tracks), len(observations)
        matched_tracks, matched_obs = set(), set()
        if n_tracks and n_obs:
            n = n_tracks + n_obs #SIze of square matrix large enough to fit new tracks for new obs
            impossible = 1e6
            cost = np.full((n, n), impossible, dtype=float)
            for i, track in enumerate(self.tracks):
                for j, obs in enumerate(observations):
                    candidate = self._candidate_cost(track, obs, projections[i])
                    if candidate is not None:
                        cost[i, j] = candidate

                #Just looped up to n_obs, n_obs+1 is next and represents no match for the track
                cost[i, n_obs + i] = self.missed_track_cost

            #Doing the same for the obs, giving them a match for new track
            for j in range(n_obs):
                cost[n_tracks + j, j] = self.new_track_cost

            #Set a square in bottom right to 0, these are new tracks with new obs
            cost[n_tracks:, n_obs:] = 0.0 

            #Make lists of optimal matches
            rows, cols = linear_sum_assignment(cost)
            for row, col in zip(rows, cols):
                if row < n_tracks and col < n_obs and cost[row, col] < impossible:
                    self.tracks[row].update(observations[col], timestamp)
                    matched_tracks.add(row)
                    matched_obs.add(col)

        #Loop through all tentative tracks that lack a matching observation
        for i, track in enumerate(self.tracks):
            if i in matched_tracks or track.confirmed:
                continue

            seen_in_image = False
            projection = projections[i]

            #Go through all raw_detections
            #If it shares class and a box center close enough to the tracks projection
            #Consider the track to have been seen in the image
            if projection is not None:
                for det in raw_detections:
                    if det['class_id'] == track.obj_class and np.linalg.norm(
                        np.asarray(det['bbox_center']) - np.asarray(projection['uv'])
                        ) <= 65.0:
                        seen_in_image = True
                        break

            #If an object wasn't seen and should have been seen
            #and missed a dealine for having been seen
            #Consider it a missed detection
            if (not seen_in_image and self._visible(projection)
                    and timestamp - track.last_visibility_check >= self.min_miss_interval):
                track.missed_expected_detection(timestamp)

        #Delete tentative tracks that have too many missing detections
        self.tracks = [t for t in self.tracks if t.confirmed or t.expected_misses < self.max_expected_misses]

        #Add new tracks for observations that had no matching track
        for j, obs in enumerate(observations):
            if j in matched_obs:
                continue
            self.tracks.append(ObjectTrack(self.next_id, obs, timestamp, self))
            self.next_id += 1

        return self.tracks