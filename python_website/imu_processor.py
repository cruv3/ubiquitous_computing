import numpy as np
from numpy.linalg import inv, norm
from mathlib import *
import scipy.signal


class IMUProcessor:

    def __init__(self, sampling, data_order={'w': 1, 'a': 2, 'm': 3}):
        super().__init__()
        # ---- parameters ----
        self.sampling = sampling
        self.dt = 1 / sampling    # second
        self.data_order = data_order

        # ---- helpers ----
        idx = {1: [0, 3], 2: [3, 6], 3: [6, 9]}
        self._widx = idx[data_order['w']]
        self._aidx = idx[data_order['a']]
        self._midx = idx[data_order['m']]
        self.init_list = None

    def initialize(self, data, noise_coefficient={'w': 100, 'a': 100, 'm': 10}):
        '''
        Algorithm initialization
        
        @param data: (,9) ndarray
        @param cut: cut the first few data to avoid potential corrupted data
        @param noise_coefficient: sensor noise is determined by variance magnitude times this coefficient
        
        Return: a list of initialization values used by EKF algorithm: 
        (gn, g0, mn, gyro_noise, gyro_bias, acc_noise, mag_noise)
        '''

        # discard the first few readings
        # for some reason they might fluctuate a lot
        w = data[:, self._widx[0]:self._widx[1]]
        a = data[:, self._aidx[0]:self._aidx[1]]
        m = data[:, self._midx[0]:self._midx[1]]

        # ---- gravity ----
        gn = -a.mean(axis=0)
        gn = gn[:, np.newaxis]
        # save the initial magnitude of gravity
        g0 = np.linalg.norm(gn)

        # ---- magnetic field ----
        mn = m.mean(axis=0)
        # magnitude is not important
        mn = self.normalized(mn)[:, np.newaxis]

        # ---- compute noise covariance ----
        avar = a.var(axis=0)
        wvar = w.var(axis=0)
        mvar = m.var(axis=0)
        print('acc var: %s, norm: %s' % (avar, np.linalg.norm(avar)))
        print('ang var: %s, norm: %s' % (wvar, np.linalg.norm(wvar)))
        print('mag var: %s, norm: %s' % (mvar, np.linalg.norm(mvar)))

        # ---- define sensor noise ----
        gyro_noise = noise_coefficient['w'] * np.linalg.norm(wvar)
        gyro_bias = w.mean(axis=0)
        acc_noise = noise_coefficient['a'] * np.linalg.norm(avar)
        mag_noise = noise_coefficient['m'] * np.linalg.norm(mvar)
        self.init_list = (gn, g0, mn, gyro_noise, gyro_bias, acc_noise, mag_noise)

    def attitudeTrack(self, data, init_list):
        # ------------------------------- #
        # ---- Initialization ----
        # ------------------------------- #
        gn, g0, mn, gyro_noise, gyro_bias, acc_noise, mag_noise = init_list
        w = data[:, self._widx[0]:self._widx[1]] - gyro_bias
        a = data[:, self._aidx[0]:self._aidx[1]]
        m = data[:, self._midx[0]:self._midx[1]]
        sample_number = np.shape(data)[0]

        # ---- data container ----
        a_nav = []
        orix = []
        oriy = []
        oriz = []

        # ---- states and covariance matrix ----
        P = 1e-10 * self.I(4)    # state covariance matrix
        q = np.array([[1, 0, 0, 0]]).T    # quaternion state
        init_ori = self.I(3)   # initial orientation

        # ------------------------------- #
        # ---- Extended Kalman Filter ----
        # ------------------------------- #

        # all vectors are column vectors

        t = 0
        while t < sample_number:

            # ------------------------------- #
            # ---- 0. Data Preparation ----
            # ------------------------------- #

            wt = w[t, np.newaxis].T
            at = a[t, np.newaxis].T
            mt = self.normalized(m[t, np.newaxis].T)

            # ------------------------------- #
            # ---- 1. Propagation ----
            # ------------------------------- #

            Ft = self.F(q, wt, self.dt)
            Gt = self.G(q)
            Q = (gyro_noise * self.dt)**2 * Gt @ Gt.T

            q = self.normalized(Ft @ q)
            P = Ft @ P @ Ft.T + Q

            # ------------------------------- #
            # ---- 2. Measurement Update ----
            # ------------------------------- #

            # Use normalized measurements to reduce error!

            # ---- acc and mag prediction ----
            pa = self.normalized(-self.rotate(q) @ gn)
            pm = self.normalized(self.rotate(q) @ mn)

            # ---- residual ----
            Eps = np.vstack((self.normalized(at), mt)) - np.vstack((pa, pm))

            # ---- sensor noise ----
            # R = internal error + external error
            Ra = [(acc_noise / np.linalg.norm(at))**2 + (1 - g0 / np.linalg.norm(at))**2] * 3
            Rm = [mag_noise**2] * 3
            R = np.diag(Ra + Rm)

            # ---- kalman gain ----
            Ht = self.H(q, gn, mn)
            S = Ht @ P @ Ht.T + R
            K = P @ Ht.T @ np.linalg.inv(S)

            # ---- actual update ----
            q = q + K @ Eps
            P = P - K @ Ht @ P

            # ------------------------------- #
            # ---- 3. Post Correction ----
            # ------------------------------- #

            q = self.normalized(q)
            P = 0.5 * (P + P.T)    # make sure P is symmertical

            # ------------------------------- #
            # ---- 4. other things ----
            # ------------------------------- #

            # ---- navigation frame acceleration ----
            conj = -self.I(4)
            conj[0, 0] = 1
            an = self.rotate(conj @ q) @ at + gn

            # ---- navigation frame orientation ----
            orin = self.rotate(conj @ q) @ init_ori

            # ---- saving data ----
            a_nav.append(an.T[0])
            orix.append(orin.T[0, :])
            oriy.append(orin.T[1, :])
            oriz.append(orin.T[2, :])

            t += 1

        a_nav = np.array(a_nav)
        orix = np.array(orix)
        oriy = np.array(oriy)
        oriz = np.array(oriz)
        return (a_nav, orix, oriy, oriz)

    def removeAccErr(self, a_nav, threshold=0.2, filter=False, wn=(0.01, 15)):
        sample_number = np.shape(a_nav)[0]
        t_start = 0
        for t in range(sample_number):
            at = a_nav[t]
            if np.linalg.norm(at) > threshold:
                t_start = t
                break

        t_end = 0
        for t in range(sample_number - 1, -1, -1):
            at = a_nav[t]
            if np.linalg.norm(at - a_nav[-1]) > threshold:
                t_end = t
                break

        an_drift = a_nav[t_end:].mean(axis=0)
        an_drift_rate = an_drift / (t_end - t_start)

        for i in range(t_end - t_start):
            a_nav[t_start + i] -= (i + 1) * an_drift_rate

        for i in range(sample_number - t_end):
            a_nav[t_end + i] -= an_drift

        if filter:
            filtered_a_nav = self.filtSignal([a_nav], dt=self.dt, wn=wn, btype='bandpass')[0]
            return filtered_a_nav
        else:
            return a_nav

    def zupt(self, a_nav, threshold):
        sample_number = np.shape(a_nav)[0]
        velocities = []
        prevt = -1
        still_phase = False

        v = np.zeros((3, 1))
        t = 0
        while t < sample_number:
            at = a_nav[t, np.newaxis].T

            if np.linalg.norm(at) < threshold:
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

        velocities = np.array(velocities)
        return velocities

    def positionTrack(self, a_nav, velocities):
        sample_number = np.shape(a_nav)[0]
        positions = []
        p = np.array([[0, 0, 0]]).T

        t = 0
        while t < sample_number:
            at = a_nav[t, np.newaxis].T
            vt = velocities[t, np.newaxis].T

            p = p + vt * self.dt + 0.5 * at * self.dt**2
            positions.append(p.T[0])
            t += 1

        positions = np.array(positions)
        return positions
    
    def normalized(self,v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v  # Rückgabe des Originalvektors, falls die Norm 0 ist
        return v / norm
    
    def process(self, imu_data):
        # IMU-Daten extrahieren
        data = np.hstack((imu_data['gyro'], imu_data['accel'], imu_data['mag'])).reshape(1, -1)
        # EKF Schritt
        a_nav, _, _, _ = self.attitudeTrack(data, self.init_list)
        
        # Beschleunigungsfehler entfernen
        #a_nav_filtered = self.removeAccErr(a_nav, filter=False)
        # ZUPT Schritt
        v = self.zupt(a_nav, threshold=0.2)
        
        # Position berechnen
        p = self.positionTrack(a_nav, v)
        return p[-1]  # Gebe die letzte berechnete Position zurück
    
    def rotate(self, q):
        qv = q[1:4, :]
        qc = q[0]
        return (qc**2 - qv.T @ qv) * self.I(3) - 2 * qc * self.skew(qv) + 2 * qv @ qv.T


    def F(self, q, wt, dt):
        w = wt.T[0]
        Omega = np.array([[0, -w[0], -w[1], -w[2]], [w[0], 0, w[2], -w[1]],
                        [w[1], -w[2], 0, w[0]], [w[2], w[1], -w[0], 0]])

        return self.I(4) + 0.5 * dt * Omega


    def G(self, q):
        q = q.T[0]
        return 0.5 * np.array([[-q[1], -q[2], -q[3]], [q[0], -q[3], q[2]],
                            [q[3], q[0], -q[1]], [-q[2], q[1], q[0]]])
    
    def skew(self, x):
        x = x.T[0]
        return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

    def Hhelper(self, q, vector):
        # just for convenience
        x = vector.T[0][0]
        y = vector.T[0][1]
        z = vector.T[0][2]
        q0 = q.T[0][0]
        q1 = q.T[0][1]
        q2 = q.T[0][2]
        q3 = q.T[0][3]

        h = np.array([
            [q0*x - q3*y + q2*z, q1*x + q2*y + q3*z, -q2*x + q1*y + q0*z, -q3*x - q0*y + q1*z],
            [q3*x + q0*y - q1*z, q2*x - q1*y - q0*z, q1*x + q2*y + q3*z, q0*x - q3*y + q2*z],
            [-q2*x + q1*y +q0*z, q3*x + q0*y - q1*z, -q0*x + q3*y - q2*z, q1*x + q2*y + q3*z]
        ])
        return 2 * h

    def H(self, q, gn, mn):
        H1 = self.Hhelper(q, gn)
        H2 = self.Hhelper(q, mn)
        return np.vstack((-H1, H2))
    
    def I(self, n):
        return np.eye(n)
    
    def filtSignal(self, data, dt=0.01, wn=10, btype='lowpass', order=1):
        res = []
        n, s = scipy.signal.butter(order, wn, fs=1 / dt, btype=btype)
        for d in data:
            d = scipy.signal.filtfilt(n, s, d, axis=0)
            res.append(d)
        return res
