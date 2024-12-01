import numpy as np
from numpy.linalg import norm


class IMUTracker:
    def __init__(self, sampling):
        """
        Initialisiert den Tracker.
        
        :param sampling: Sampling-Rate der IMU (Hz).
        """
        self.sampling = sampling
        self.dt = 1 / sampling  # Zeitschritt in Sekunden

    def removeAccErr(self, a_nav, threshold=0.2):
        """
        Entfernt Drift in den Beschleunigungsdaten, indem Anfangs- und Endabschnitte korrigiert werden.
        
        :param a_nav: Beschleunigungsdaten (raw output vom Kalman-Filter).
        :param threshold: Schwellwert zur Erkennung von Bewegung.
        :return: Korrigierte Beschleunigungsdaten.
        """
        sample_number = a_nav.shape[0]

        # Anfang der Bewegung finden
        t_start = 0
        for t in range(sample_number):
            if norm(a_nav[t]) > threshold:
                t_start = t
                break

        # Ende der Bewegung finden
        t_end = sample_number - 1
        for t in range(sample_number - 1, -1, -1):
            if norm(a_nav[t] - a_nav[-1]) > threshold:
                t_end = t
                break

        # Drift korrigieren
        an_drift = a_nav[t_end:].mean(axis=0)
        an_drift_rate = an_drift / (t_end - t_start)

        for i in range(t_end - t_start):
            a_nav[t_start + i] -= (i + 1) * an_drift_rate

        for i in range(sample_number - t_end):
            a_nav[t_end + i] -= an_drift

        return a_nav

    def zupt(self, a_nav, threshold=0.2):
        """
        Zero Velocity Update (ZUPT) Algorithmus, um Drift zu reduzieren.
        
        :param a_nav: Beschleunigungsdaten im Navigationsrahmen.
        :param threshold: Schwellwert für die stationäre Erkennung.
        :return: Geschwindigkeitsdaten.
        """
        sample_number = a_nav.shape[0]
        velocities = []
        prevt = -1
        still_phase = False

        v = np.zeros((3, 1))
        t = 0
        while t < sample_number:
            at = a_nav[t, np.newaxis].T

            if norm(at) < threshold:
                if not still_phase:
                    predict_v = v + at * self.dt
                    v_drift_rate = predict_v / (t - prevt)
                    for i in range(t - prevt - 1):
                        velocities[prevt + 1 + i] -= (i + 1) * v_drift_rate.T[0]

                v = np.zeros((3, 1))
                prevt = t
                still_phase = True
            else:
                v = v + at * self.dt
                still_phase = False

            velocities.append(v.T[0])
            t += 1

        return np.array(velocities)

    def positionTrack(self, a_nav, velocities):
        """
        Integration von Beschleunigungs- und Geschwindigkeitsdaten zur Positionsberechnung.
        
        :param a_nav: Beschleunigungsdaten.
        :param velocities: Geschwindigkeitsdaten.
        :return: Positionen in 3D.
        """
        sample_number = a_nav.shape[0]
        positions = []
        p = np.zeros((3, 1))

        for t in range(sample_number):
            at = a_nav[t, np.newaxis].T
            vt = velocities[t, np.newaxis].T
            p = p + vt * self.dt + 0.5 * at * self.dt**2
            positions.append(p.T[0])

        return np.array(positions)

    def process(self, a_nav, threshold=0.2):
        """
        Entfernt Drift, führt ZUPT durch und berechnet die Position.
        
        :param a_nav: Navigationsbeschleunigungsdaten.
        :param threshold: Schwellwert für die stationäre Erkennung.
        :return: Positionen in 3D.
        """
        a_nav_corrected = self.removeAccErr(a_nav, threshold)
        velocities = self.zupt(a_nav_corrected, threshold)
        positions = self.positionTrack(a_nav_corrected, velocities)
        return positions
