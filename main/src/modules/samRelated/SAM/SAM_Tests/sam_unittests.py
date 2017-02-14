from SAM.SAM_Tests import modelsTest

modelDriverList = [('Actions3', 'SAMDriver_ARWin')]
dataCollectList = [True, False]
randomRecallList = [True]
probRecallList = [0.01, 0.1, 0.3]
bufferLenList = [10, 30, 50]
recencyList = [5, 15, 30]
transmissionDelayList = [0.02, 0.05]

param_list = []
for a in modelDriverList:
    for b in dataCollectList:
        for c in randomRecallList:
            for d in probRecallList:
                for e in bufferLenList:
                    for f in recencyList:
                        for g in transmissionDelayList:
                            param_list.append((a[0], a[1], b, c, d, e, f, g))
print len(param_list), 'tests queued'


def test_generator():
    for params in param_list:
        yield check_em, params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7]


def check_em(modelName, driverName, datacollectionOnly, randomRecall,
             probRecall, bufferLen, recency, transmissionDelay):
    assert modelsTest.test_modelWithParams(modelName, driverName, datacollectionOnly, randomRecall,
                                           probRecall, bufferLen, recency, transmissionDelay)
