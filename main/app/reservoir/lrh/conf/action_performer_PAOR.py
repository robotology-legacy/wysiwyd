# -*- coding: utf-8 -*-
"""
Created on 11 janv. 2012

@author: Xavier HINAUT
xavier.hinaut #/at\# inserm.fr

"""

import mdp
import Oger
import numpy as np
import common_PAOR as common


def get_closed_class_words():
    """
    list of closed class words
    """
    return closed_class_wordsAP
    
def meaning_stim_to_meaning_code(stim, ocw_array, l_m_elt, verbose=False):
    """
    inputs:
        -stim: meaning stim for one time step
        
    
    # action and relation is a full predicate (with max_nr_elt_pred element describing it)
    
    For 8 open class words maximum: so 2 actions/relations with 4 nouns for each (predicate, focus, o1, o2)
    [N1(1) N1(2) N2(1) N2(2) N3(1) N3(2) N4(1) N4(2) N5(1) N5(2) N6(1) N6(2) N7(1) N7(2) N8(1) N8(2)]
    trumpet below guitar,guitar right violin; #FPOR order
    below trumpet guitar,right guitar violin; #FPOR order
    below     the     guitar     and     right     to     the     violin     the     trumpet     is 
     P1               O1                 P2                        O2                F1&F2
     _1-P1            _2-O1              _3-P2                     _4-O2             _5-F1&_5-F2   # '_' indicates an open class word
    1,0,0,0,0,0,0,0,  0,0,1,0,0,0,0,0,   0,0,0,0,1,0,0,0,          0,0,0,0,0,0,1,0,  0,1,0,0,0,1,0,0  # coded in the order PFOR (Predicate Focus Object Recipient)
    """
    ocws = ocw_array[:]
    ocws.reverse()
    meanings = []
    PAOR = []
    for i_m in range(0,max_nr_actionrelation):
        meaning = []
        for i_elt_pred in range(0,len(elt_pred)):
            meaning.append(None)
        meanings.append(meaning)
    if verbose:
        print "  initial meanings:", meanings
    # for each signal in stim/output
    for i in range(len(l_m_elt)):
        if stim[i]==1:
            m_elt = l_m_elt[i]
            idx_ocw = int(m_elt[1]) # looking at '#' in '_#-P1'
            try:
                word = ocw_array[idx_ocw-1]
            except:
                word = '_X_'
            e_p = str(m_elt[3]) # looking at 'X' in '_1-X1'
            idx_elt_pred = elt_pred.index(e_p)
            idx_meaning = int(m_elt[4]) # looking at '#' in '_1-P#'
            meanings[idx_meaning-1][idx_elt_pred] = word
            tmp=[]
            tmp.append(word)
            tmp.append(m_elt.split("-",1)[1])
            PAOR.append(tmp);
            if verbose:
                print "--i=", i
                print "  m_elt (l_m_elt[i])=", m_elt
                print "  idx_ocw:", idx_ocw
                print "  word:", word
                print "  e_p:", e_p
                print "  idx_meaning:", idx_meaning
                print "  meanings:", meanings
    for i_m in range(0,max_nr_actionrelation):
        while True:
            try:
                meanings[i_m].remove(None)
            except:
                break				
    while True:
        try:
            meanings.remove([])
        except:
            break
	# create proper meaning
    isUsed = []
    properPAOR = []
    for jj in range(0,len(meanings)):
        for ii in range(0,len(meanings[jj])):
            isUsed.append(True)
    for jj in range(0,len(meanings)):
        for ii in range(0,len(meanings[jj])):
            missing = True
            for kk in range(0,len(PAOR)):
                if missing and PAOR[kk][0] ==  meanings[jj][ii] and isUsed[kk]:
                    properPAOR.append(PAOR[kk][1])
                    isUsed[kk] = False
                    missing = False
    meanings.append(properPAOR)
    return meanings

def convert_one_output_activity_in_meaning(out_act, ocw_array, l_m_elt, thres=0.6):
    # threshold the output
    act_thres = (out_act>thres)
    # get the code from the last time time activity
    meanings = meaning_stim_to_meaning_code(stim=act_thres[-1,:], ocw_array=ocw_array, l_m_elt=l_m_elt)
    # get meanings with code and l_ocw_array
#    meanings = code_to_meanings(code, ocw_array)
    return meanings

def convert_l_output_activity_in_meaning(l_out_act, l_ocw_array, l_m_elt):
    l_meanings = [] # list of meanings
    for i_out in range(len(l_out_act)):
        meanings = convert_one_output_activity_in_meaning(l_out_act[i_out], l_ocw_array[i_out], l_m_elt=l_m_elt)
        l_meanings.append(meanings)
    return l_meanings

def main(path_file_in, path_file_out, plot=False, fast=False, keep_internal_states=False, verbose=False):
    import os
    #sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/..")
    #print "path   ", os.path.dirname(os.path.abspath(__file__))+"/.."
    current_directory = os.path.dirname(os.path.abspath(__file__))
    parent_directory = os.path.dirname(current_directory)
    sys.path.append(parent_directory)
    import io_language_coding as CtIolangcod
    
    
    # Definning parameters of stimulus (in a dictionary)
    d = {}
    d['act_time'] = 5#2#1#5#10#2
    d['pause'] = True#False
    d['suppl_pause_at_the_end'] = 1*d['act_time']
    d['initial_pause'] = False#True#False#False
    d['offset'] = True#False#True
    
    # Parameters for reservoir
    N = 500#500#500#1000 #100
    sr = 1#3#3#2#1
    iss = 0.25#0.01#1
    leak = 0.25/float(d['act_time'])#0.75/float(d['act_time'])#0.75/2.#0.5#0.05
    
    ## Random parameters
    seed = 5
    if seed is not None:
        mdp.numx.random.seed(seed)
        np.random.seed(seed)
        
    [train_data_txt, test_data_txt, sent_form_info_test] = common.extract_data_io(path_file=path_file_in)
    train_corpus, train_meaning  = common.txt2corpus_and_meaning(train_txt=train_data_txt)

    test_corpus = test_data_txt
    
    # making the list of constructions (refering to "construction grammar"), a construction is a sentence without its open class words (Nouns and Verbs)
    (l_construction_train, l_ocw_array_train, construction_words) = common.get_and_remove_ocw_in_corpus(corpus=train_corpus, _OCW='X', l_closed_class=get_closed_class_words())

    (l_construction_test, l_ocw_array_test, construction_words_test) = common.get_and_remove_ocw_in_corpus(corpus=test_corpus, _OCW='X', l_closed_class=get_closed_class_words())
    if construction_words!=construction_words_test:
        raise Exception, "The construction words are not the same for the train constructions and the test constructions. So the coding of sentences will be different and should provoque a future problem."

    
    #################################################
    ## Generating all the sentence stimulus (in order to have the same length for each sentence)

    l_full_const = l_construction_train + l_construction_test
    slice_test = slice(len(l_construction_train),len(l_construction_train)+len(l_construction_test))
        
    slice_train = slice(0,len(l_construction_train))
    (stim_full_data, l_full_offset) = CtIolangcod.generate_stim_input_nodic(l_data=l_full_const,
#                            act_time=d['act_time'], subset=None, l_input=None,
                            act_time=d['act_time'], subset=None, l_input=construction_words,
                            l_nr_word=None, mult=None, full_time=None,
                            with_offset=d['offset'], pause=d['pause'], initial_pause=d['initial_pause'],
                            suppl_pause_at_the_end=d['suppl_pause_at_the_end'], verbose=False)
    stim_sent_train = stim_full_data[slice_train]

    stim_sent_test = stim_full_data[slice_test]


    #################################################
    ## Generating all the meaning stimulus 
    #################################################

    l_m_elt = common.get_meaning_coding(max_nr_ocw=max_nr_ocw, max_nr_actionrelation=max_nr_actionrelation, elt_pred=elt_pred)

    (stim_mean_train, l_meaning_code_train) = common.generate_meaning_stim(l_data=train_meaning,
           l_ocw_array=l_ocw_array_train, full_time=stim_sent_train[0].shape[0],
           l_m_elt=l_m_elt, l_offset=l_full_offset[slice_train], verbose=False,
           initial_pause=d['initial_pause'], pause=d['pause'], act_time=d['act_time'])
        
    ## Defining reservoir, readout and flow
    reservoir = Oger.nodes.LeakyReservoirNode(output_dim = N, spectral_radius = sr, input_scaling =iss, nonlin_func = np.tanh, leak_rate = leak)
    read_out = mdp.nodes.LinearRegressionNode(use_pinv=True, with_bias=True)
    flow = mdp.Flow([reservoir, read_out])
    if keep_internal_states:
        Oger.utils.make_inspectable(mdp.Flow)
        
    ## Trainning and testing
    print "Train and test"
    if not fast:
        (states_out_train, internal_states_train, internal_outputs_train, neuron_states_train) = \
            common._teach_and_test_flow(inputs_train_set=stim_sent_train, teacher_outputs_train_set=stim_mean_train, inputs_test_set=stim_sent_train, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)
    else:
        raise Exception, "have to define what to do for fast mode"
    ## test set not train set

    (states_out_test, internal_states_test, internal_outputs_test, neuron_states_test) = \
        common._test_flow(inputs_test_set=stim_sent_test, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)

    if verbose:
        for i in range(len(stim_mean_train)):
            print "len(stim_mean_train)", len(stim_mean_train)
            print "len(l_meaning_code_train)", len(l_meaning_code_train)
            print l_meaning_code_train[i]
            print (stim_mean_train[0]==stim_mean_train[i])
            print (l_meaning_code_train[0]==l_meaning_code_train[i])
    
    ## Writting output meaning

    l_recovered_meaning_test = convert_l_output_activity_in_meaning(l_out_act=states_out_test, l_ocw_array=l_ocw_array_test, l_m_elt=l_m_elt)

    if verbose:
        print "l_recovered_meaning_test", l_recovered_meaning_test
    l_final_mean_test = []
    for meanings in l_recovered_meaning_test:
        current_meanings = ""
        if verbose:
            print "meanings", meanings
        for i_m in range(len(meanings)):
            # if verbose:
            # print " i_m:",i_m
            #  print " meanings[i_m]:",meanings[i_m]
            if i_m>0:
                current_meanings+=','
            current_meanings+=" ".join(meanings[i_m])
        l_final_mean_test.append(current_meanings)
    print ""
    print "**********************************************"
    print " *** RECOGNIZED MEANINGS *** "
    for elt in l_final_mean_test:
        print str(elt)
    print "**********************************************"

    ## Writting sentences to output file
    print " *** Writting to output file ... *** "
    #ecrire une seule ligne simple dans un fichier la phrase attendue en mode test
    common.write_list_in_file(l=l_final_mean_test, file_path=path_file_out)   
    
    print " *** ... Writting done ***"
    print "**********************************************"

    ## Plot
    if plot:
        print " *** Plotting to output file ... *** "
        import oct2011.plotting as plotting
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_train",
                                    array_=states_out_train, titles_subset=l_meaning_code_train,
#                                        legend_=l_m_elt, plot_slice=None, title="", subtitle="")
                                    legend_=None, plot_slice=None, title="", subtitle="")
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_train_sent",
                                    array_=states_out_train, titles_subset=train_meaning,
#                                        legend_=l_m_elt, plot_slice=None, title="", subtitle="")
                                    legend_=None, plot_slice=None, title="", subtitle="")
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_test",
                                    array_=states_out_test, titles_subset=l_final_mean_test,
#                                        legend_=l_m_elt, plot_slice=None, title="", subtitle="")
                                    legend_=None, plot_slice=None, title="", subtitle="")   
    
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/intern_states_test", array_=internal_states_test, titles_subset=None, plot_slice=None, title="", subtitle="")

        print " *** ... Plotting to output file done *** "
        print "**********************************************"
        return l_final_mean_test 

if __name__ == '__main__':
    import sys
    # global variables
    corpusFileAP = sys.argv[1]
    fileResult = sys.argv[2]
    closed_class_wordsAP = sys.argv[3].split(',') # ['after', 'and', 'before', 'to', 'the', 'slowly', 'quickly', 'was', 'with', 'that', 'for', 'a', 'now']
    max_nr_ocw = int(sys.argv[4])              #10
    max_nr_actionrelation = int(sys.argv[5])    #4
    elt_pred= sys.argv[6].split(',')     #['P','A','O','R','V']
    
    main(path_file_in=corpusFileAP,
             path_file_out=fileResult)

    print "*********END OF PROGRAM********"

