import numpy as np

def random_data_split(Y, percentage=[0.5,0.5]):
    N = Y.shape[0]
    N_1 = np.ceil(N*percentage[0])
    N_2 = np.floor(N*percentage[1])
    assert(N==N_1+N_2)
    perm = np.random.permutation(N)
    inds_1 = perm[0:N_1]
    inds_2 = perm[N_1:N_1+N_2]
    return Y[inds_1,:], Y[inds_2,:], inds_1, inds_2



class SURFProcessor():
    def __init__(self, imgHNew, imgWNew,n_clusters=20,SURFthresh=500,crop_thresholds=(40,160,40,160),magnify=1):
        self.imgHNew = imgHNew
        self.imgWNew = imgWNew
        self.n_clusters = n_clusters
        self.SURFthresh = SURFthresh
        self.crop_thresholds = crop_thresholds
        self.magnify = magnify # Magnify photos (scale up) before extracting SURF

        self._is_trained = False

    def _find_surf(self,Y,thresh=None,h=None,w=None,limits=None,magnify=None):
        import sys
        if thresh is None: thresh=self.SURFthresh
        if h is None: h = self.imgHNew
        if w is None: w = self.imgWNew
        if limits is None: limits = self.crop_thresholds
        if magnify is None: magnify = self.magnify

        print '# Finding SURF features from ' + str(Y.shape[0]) + ' images...',
        sys.stdout.flush()
        import cv2
        surf = cv2.SURF(thresh)
        descriptors = []
        desclabels = []
        for i in range(Y.shape[0]):
            if magnify is not 1:
                y_tmp = cv2.resize(Y[i,:].reshape(h,w),(h*magnify,w*magnify)).flatten()
                kp, des = surf.detectAndCompute(np.uint8(y_tmp.reshape(h*magnify, w*magnify)),None)
            else:
                y_tmp = Y[i,:]
                kp, des = surf.detectAndCompute(np.uint8(y_tmp.reshape(h, w)),None)

            for j in range(len(kp)):
                if (limits is not None) and (kp[j].pt[0] < limits[0] or kp[j].pt[0]>limits[1] or kp[j].pt[1] < limits[2] or kp[j].pt[1]>limits[3]):
                    continue
                desclabels.append(i)
                descriptors.append(des[j])

        descriptors = np.array(descriptors)
        desclabels = np.array(desclabels)
        print ' Found ' + str(len(desclabels)) + ' features.'
        return descriptors, desclabels

    def _make_BoW(self,N, c_trainPredict, desclabels, n_clusters=None):
        if n_clusters is None: n_clusters = self.n_clusters
        print('# Making BoW...')
        features = []
        for i in range(N):
            feature_counts = c_trainPredict[desclabels==i]
            feature_vector = np.zeros(n_clusters)
            for j in range(n_clusters):
                feature_vector[j] = np.where(feature_counts==j)[0].shape[0]
            features.append(feature_vector)

        return np.sqrt(np.array(features))



    def make_SURF_BoW(self,Y,normalize=True,imgHNew=None, imgWNew=None,n_clusters=None,SURFthresh=None,crop_thresholds=None):
        if imgHNew is None: imgHNew = self.imgHNew;
        if imgWNew is None: imgWNew = self.imgWNew;
        if n_clusters is None: n_clusters = self.n_clusters
        if SURFthresh is None: SURFthresh = self.SURFthresh
        if crop_thresholds is None: crop_thresholds = self.crop_thresholds
        self.normalize = normalize

        from sklearn.cluster import KMeans
        from scipy.spatial import distance

        #### APPROACH 1: Bag of feature approach:
        # Cluster all descriptors (for each image) and then replace each image with a histogram
        # saying how many descriptors it has that fall in cluster K. So, K dimensional feature vec.
        
        # Thresh to exclude corners...

        ## TRAINING DATA
        descriptors, desclabels = self._find_surf(Y,SURFthresh,imgHNew, imgWNew,crop_thresholds)

        # Agglomeratice only: Define the structure A of the data. Here a 10 nearest neighbors
        #from sklearn.neighbors import kneighbors_graph
        #connectivity = kneighbors_graph(descriptors, n_neighbors=10, include_self=False)

        # Compute clustering
        #ward = AgglomerativeClustering(n_clusters=n_clusters, connectivity=connectivity,linkage='ward',compute_full_tree=True).fit(descriptors)
        #Y_ = ward.labels_

        c = KMeans(n_clusters=n_clusters, max_iter=10000,random_state=1)

        c_trainFit = c.fit(descriptors)
        c_trainPredict = c_trainFit.predict(descriptors)

        #Equiv:
        #c = KMeans(n_clusters=n_clusters, max_iter=2000,random_state=1)
        #c_trainFitPredict=c.fit_predict(descriptors)

        #plt.plot(c_trainPredict, 'x')
        Z = self._make_BoW(Y.shape[0], c_trainPredict, desclabels, n_clusters)
        self.c_trainFit = c_trainFit

        #plt.matshow(Z);  plt.colorbar(); plt.gca().set_aspect('normal');plt.draw()
        #plt.matshow(mySAMpy.L);  plt.gca().set_aspect('normal');plt.draw()

        if normalize:
            Zmean = Z.mean()
            Zn = Z - Zmean
            Zstd = Zn.std()
            Zn /= Zstd
            self.Zmean = Zmean
            self.Zstd = Zstd
            Z = Zn

        self._is_trained = True
        return Z.copy(), descriptors, desclabels, c_trainFit



    def make_SURF_BoW_test(self,Ytest):
        # SURF for test data
        assert(self._is_trained, "First you have to do a train BoW.")
        descriptors, desclabels = self._find_surf(Ytest)
        c_testPredict = self.c_trainFit.predict(descriptors)
        Ztest = self._make_BoW(Ytest.shape[0], c_testPredict, desclabels, self.n_clusters)
        if self.normalize:
            Ztestn = Ztest - self.Zmean
            Ztestn /= self.Zstd
            Ztest = Ztest

        return Ztest.copy()
