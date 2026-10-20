#!/usr/bin/env python3

import numpy as np


class ObjectTrack:

    def __init__(self, track_id, obj_class, position, timestamp, min_observations):
        self.id = track_id
        self.obj_class = obj_class
        self.position = np.array(position, dtype=float)
        self.observations = 1
        self.last_seen = timestamp
        self.min_observations = min_observations

    @property
    def status(self):
        return 'confirmed' if self.observations >= self.min_observations else 'tentative'

    def update(self, position, timestamp):
        position = np.array(position, dtype=float)

        # temporary, TODO try to replace with linear kalman
        alpha = 0.3
        self.position = (
            (1.0 - alpha) * self.position
            + alpha * position
        )
        self.observations += 1
        self.last_seen = timestamp


class ObjectTracker:

    def __init__(self, max_distance=0.5, min_observations=5):
        self.tracks = []
        self.next_id = 1
        self.max_distance = max_distance
        self.min_observations = min_observations

    def confirmed_tracks(self):
        """Only these tracks will be eligible for navigation publishing later."""
        return [track for track in self.tracks if track.status == 'confirmed']

    def update(self, observations, timestamp):

        candidates = []
        for obs_idx, observation in enumerate(observations):
            for track_idx, track in enumerate(self.tracks):
                if observation['class_id'] != track.obj_class:
                    continue

                distance = np.linalg.norm(
                    observation['position'] - track.position
                )
                if distance <= self.max_distance:
                    candidates.append((distance, obs_idx, track_idx))

        candidates.sort(key=lambda candidate: candidate[0])
        matched_observations = set()
        matched_tracks = set()

        #One observation can update at most one track, and vice versa.
        for distance, obs_idx, track_idx in candidates:
            if obs_idx in matched_observations or track_idx in matched_tracks:
                continue

            observation = observations[obs_idx]
            track = self.tracks[track_idx]
            track.update(observation['position'], timestamp)

            matched_observations.add(obs_idx)
            matched_tracks.add(track_idx)

        for obs_idx, observation in enumerate(observations):
            if obs_idx in matched_observations:
                continue

            new_track = ObjectTrack(
                track_id=self.next_id,
                obj_class=observation['class_id'],
                position=observation['position'],
                timestamp=timestamp,
                min_observations=self.min_observations
            )
            self.tracks.append(new_track)
            self.next_id += 1

        return self.tracks
