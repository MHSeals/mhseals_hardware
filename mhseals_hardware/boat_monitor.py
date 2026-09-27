"""Bounded, thread-safe telemetry and trial analysis, independent of ROS/UI."""
from collections import deque
from copy import deepcopy
import math
import threading
import time


class BoatMonitor:
    def __init__(self, stale_after=2.0, clock=time.monotonic):
        self.stale_after = stale_after
        self.clock = clock
        self.lock = threading.RLock()
        self.sensors = {}
        self.edges = {}
        self.history = deque(maxlen=12000)
        self.events = deque(maxlen=12)
        self.results = deque(maxlen=100)

    def observe(self, name, values=None, issue='', stamp_age=None):
        now = self.clock()
        values = dict(values or {})
        for key, value in values.items():
            if isinstance(value, float) and not math.isfinite(value):
                values[key] = None
                issue = issue or 'non-finite measurement'
        with self.lock:
            entry = self.sensors.setdefault(name, {'times': deque(maxlen=300), 'count': 0})
            entry['times'].append(now)
            entry['count'] += 1
            entry.update(values=values, issue=issue, stamp_age=stamp_age)
            if name == 'odometry':
                self.history.append((now, deepcopy(values), issue))

    def transform(self, parent, child, stamp_age=0.0, static=False, authority=''):
        with self.lock:
            self.edges[(parent, child, authority)] = (self.clock(), stamp_age, static)
            # Retain static transforms but bound stale dynamic discovery churn.
            if len(self.edges) > 300:
                oldest = min(self.edges, key=lambda key: self.edges[key][0])
                del self.edges[oldest]

    def event(self, message):
        with self.lock:
            self.events.append((self.clock(), str(message)))

    def snapshot(self, required_tf=(('map', 'odom'), ('odom', 'base_link'))):
        now = self.clock()
        with self.lock:
            sensors = {}
            for name, entry in self.sensors.items():
                times = [t for t in entry['times'] if now - t <= 5]
                age = now - entry['times'][-1]
                stamp_age = entry['stamp_age']
                issue = entry['issue']
                if stamp_age is not None:
                    stamp_age += age
                    if stamp_age < -0.25:
                        issue = issue or 'timestamp is in the future'
                    elif stamp_age > self.stale_after:
                        issue = issue or 'stale message timestamp'
                status = 'STALE' if age > self.stale_after else ('WARN' if issue else 'LIVE')
                sensors[name] = dict(age=age, status=status, issue=issue,
                                     stamp_age=stamp_age, count=entry['count'],
                                     hz=(len(times) - 1) / (times[-1] - times[0])
                                     if len(times) > 1 and times[-1] > times[0] else 0,
                                     values=deepcopy(entry['values']))
            edges = dict(self.edges)
            tf = []
            for source, target in required_tf:
                graph = {}
                parents = {}
                for (parent, child, authority), (received, age, static) in edges.items():
                    if static or (now - received <= self.stale_after and
                                  -.25 <= age + now - received <= self.stale_after):
                        graph.setdefault(parent, []).append(child)
                        parents.setdefault(child, set()).add((parent, authority))
                pending = [(source, ())]
                found = None
                while pending:
                    frame, path = pending.pop()
                    if frame in path:
                        continue
                    if frame == target:
                        found = path + (frame,)
                        break
                    pending.extend((child, path + (frame,)) for child in graph.get(frame, []))
                conflicts = [child for child in (found or ()) if len(parents.get(child, ())) > 1]
                # A cycle connected to a required frame is never a valid TF tree.
                visited = set()
                def cyclic(frame, path):
                    if frame in path:
                        return True
                    if frame in visited:
                        return False
                    visited.add(frame)
                    return any(cyclic(child, path | {frame}) for child in graph.get(frame, []))
                cycle = cyclic(source, set())
                status = 'MISSING' if found is None else ('WARN' if conflicts or cycle else 'LIVE')
                detail = ('cycle detected' if cycle else 'multiple parents/publishers: ' + ', '.join(conflicts)
                          if conflicts else ' -> '.join(found) if found else 'no fresh directed chain')
                tf.append(dict(source=source, target=target, status=status, detail=detail))
            recent = [(t, v) for t, v, issue in self.history if now - t <= 10 and not issue]
            step = max(1, len(recent) // 32)
            trends = {key: [v[key] for _, v in recent[::step] if v.get(key) is not None][-32:]
                      for key in ('vx', 'vy', 'wz')}
            return dict(sensors=sensors, tf=tf, events=list(self.events), trends=trends,
                        results=deepcopy(list(self.results)))

    def summarize_trial(self, axis, direction, baseline_start, command_start, command_end):
        """Describe measured response, never claim thrust or navigation calibration."""
        component = {'surge': 'vx', 'sway': 'vy', 'yaw': 'wz'}[axis]
        with self.lock:
            samples = list(self.history)
        baseline = [(t, v) for t, v, issue in samples
                    if baseline_start <= t < command_start and not issue and component in v]
        driven = [(t, v) for t, v, issue in samples
                  if command_start <= t <= command_end and not issue and component in v]
        result = dict(axis=axis, direction=direction, samples=len(driven),
                      verdict='INCONCLUSIVE', detail='insufficient fresh odometry')
        if len(driven) >= 5 and len(baseline) >= 3:
            gaps = [driven[0][0] - command_start, command_end - driven[-1][0]]
            gaps += [b[0] - a[0] for a, b in zip(driven, driven[1:])]
            frames = {(v.get('frame'), v.get('child')) for _, v in baseline + driven}
            if max(gaps) > self.stale_after or len(frames) != 1:
                result['detail'] = 'odometry gap or frame change during trial'
            else:
                drift = sum(v[component] for _, v in baseline) / len(baseline)
                mean = sum(v[component] for _, v in driven) / len(driven)
                delta = mean - drift
                verdict = ('LOW RESPONSE' if abs(delta) < .03 else
                           'SIGN MATCH' if delta * direction > 0 else 'SIGN MISMATCH')
                result.update(verdict=verdict, baseline=drift, mean=mean,
                              response=delta, peak=max(abs(v[component]) for _, v in driven),
                              detail='body-frame velocity response; not a calibration pass')
                result['mean_velocity'] = {
                    key: sum(v.get(key, 0.) for _, v in driven) / len(driven)
                    for key in ('vx', 'vy', 'wz')}
                result['max_sample_gap'] = max(gaps)
                if all(key in driven[0][1] and key in driven[-1][1] for key in ('x', 'y')):
                    result['displacement'] = {
                        'frame': driven[0][1].get('frame'),
                        'dx': driven[-1][1]['x'] - driven[0][1]['x'],
                        'dy': driven[-1][1]['y'] - driven[0][1]['y']}
        with self.lock:
            self.results.append(result)
        return result


def finite_values(values):
    return all(math.isfinite(value) for value in values)
