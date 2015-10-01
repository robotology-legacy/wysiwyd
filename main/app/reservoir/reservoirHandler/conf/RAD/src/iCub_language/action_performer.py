# -*- coding: utf-8 -*-
"""
Created on 11 janv. 2012

@author: Xavier HINAUT
xavier.hinaut #/at\# inserm.fr
"""

import mdp
import Oger
import numpy as np
import common
greg = []

def get_closed_class_words():
    """
    list of closed class words
    """

#    return ['and', 'is', 'of', 'the', 'to', '.']
#    return ['after', 'before', 'it', 'on', 'the', '.']
    return ['after', 'and', 'before', 'it', 'on', 'in', 'to', 'the', 'then', 'you', '.', 'my', 'red', 'blue','please', 'twice', 'two', 'steps', 'do', 'a', 'u-turn', 'both', 'objects', 'having', 'again','afterwards', 'time','firstly','first', 'over', 'back', 'at','followed','once']

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
    l_code = []
    meanings = []
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
            if verbose:
                print "  i=", i
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
    return meanings

#def code_to_meanings(code, l_ocw_array):
#    meanings = []
#    for i in max_nr_actionrelation:
#        meanings.append([])
#    for i_action in range(1,max_nr_actionrelation+1):
#        if 'P'+
#    for elt in code:
        

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

def main(path_file_in, path_file_out, sentence_to_meaning=True, plot=False, fast=False, keep_internal_states=False, verbose=False):
    import sys,os
    #sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/..")
    #print "path   ", os.path.dirname(os.path.abspath(__file__))+"/.."
    current_directory = os.path.dirname(os.path.abspath(__file__))
    parent_directory = os.path.dirname(current_directory)
    sys.path.append(parent_directory)
    print "sys.path.append(parent_directory)", parent_directory
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
        
    [train_data_txt, test_data_txt, sent_form_info_test] = common.extract_data_io(path_file=path_file_in, sentence_to_meaning=sentence_to_meaning)
#    print "**************************"
#    print "train data_txt", train_data_txt
#    print "test data_txt", test_data_txt
#    print "sent_form_info_test", sent_form_info_test
    train_corpus, train_meaning  = common.txt2corpus_and_meaning(train_txt=train_data_txt)
    
    if sentence_to_meaning:
        test_corpus = test_data_txt
    else:
        test_meaning = test_data_txt
    
    # making the list of constructions (refering to "construction grammar"), a construction is a sentence without its open class words (Nouns and Verbs)
    (l_construction_train, l_ocw_array_train, construction_words) = common.get_and_remove_ocw_in_corpus(corpus=train_corpus, _OCW='X', l_closed_class=get_closed_class_words())
#    print "**************************"
#    print "l_construction_train", l_construction_train
#    print "l_ocw_array_train", l_ocw_array_train
    if sentence_to_meaning:
        (l_construction_test, l_ocw_array_test, construction_words_test) = common.get_and_remove_ocw_in_corpus(corpus=test_corpus, _OCW='X', l_closed_class=get_closed_class_words())
#        print "l_construction_test", l_construction_test
        if construction_words!=construction_words_test:
            raise Exception, "The construction words are not the same for the train constructions and the test constructions. So the coding of sentences will be different and should provoque a future problem."
    else:
        # check if a special form of sentence is requested (canonical or non-canonical form)
        # i.e. check if there is at least one element that is not None
        print ""
        print "*** Managing sentence form ... ***"
        print "sent_form_info_test:", sent_form_info_test
        # if all sentence information is None (not attributed)
        if all(elt is None for elt in sent_form_info_test):
            # generate default form of sentence
            l_ocw_array_test = generate_l_ocw_array_in_canonical_order(l_meaning=test_meaning)
        # if at least one element is not None
        else:
            # call specific method to deal with the specified order of each meanings in the list
            l_ocw_array_test = generate_l_ocw_array_in_specified_order(l_meaning=test_meaning, l_sent_form = sent_form_info_test)
        print "*** ... sentence form managed ***"
#    print "l_ocw_array_test", l_ocw_array_test
    
    #################################################
    ## Generating all the sentence stimulus (in order to have the same length for each sentence)
    #################################################
    if sentence_to_meaning:
        ## Generate the stimulus input for train and test data
        l_full_const = l_construction_train + l_construction_test
#        slice_train = slice(0,len(l_construction_train))
        slice_test = slice(len(l_construction_train),len(l_construction_train)+len(l_construction_test))
#        print "slice_train", slice_train
        print "slice_test", slice_test
    else:
        l_full_const = l_construction_train
        
    slice_train = slice(0,len(l_construction_train))
    (stim_full_data, l_full_offset) = CtIolangcod.generate_stim_input_nodic(l_data=l_full_const,
#                            act_time=d['act_time'], subset=None, l_input=None,
                            act_time=d['act_time'], subset=None, l_input=construction_words,
                            l_nr_word=None, mult=None, full_time=None,
                            with_offset=d['offset'], pause=d['pause'], initial_pause=d['initial_pause'],
                            suppl_pause_at_the_end=d['suppl_pause_at_the_end'], verbose=False)
    stim_sent_train = stim_full_data[slice_train]
    if sentence_to_meaning:
        stim_sent_test = stim_full_data[slice_test]


    #################################################
    ## Generating all the meaning stimulus 
    #################################################
#    l_m_elt = common.get_meaning_coding()
#    elt_pred=['P','F','O','R']
    l_m_elt = common.get_meaning_coding(max_nr_ocw=8, max_nr_actionrelation=2, elt_pred=elt_pred)
#    print ""
#    print "*** Generating meaning for train set ... ***"
#    (stim_mean_train, l_meaning_code_train) = common.generate_meaning_stim(l_data=train_meaning, l_ocw_array=l_ocw_array_train, full_time=stim_sent_train[0].shape[0], l_m_elt=l_m_elt, verbose=False)
    (stim_mean_train, l_meaning_code_train) = common.generate_meaning_stim(l_data=train_meaning,
           l_ocw_array=l_ocw_array_train, full_time=stim_sent_train[0].shape[0],
           l_m_elt=l_m_elt, l_offset=l_full_offset[slice_train], verbose=False,
           initial_pause=d['initial_pause'], pause=d['pause'], act_time=d['act_time'])
#    print "*** ... meaning generated for train set ***"
#    print "l_m_elt", l_m_elt
#    print "stim_mean_train[0].shape", stim_mean_train[0].shape
#    print "l_meaning_code_train", l_meaning_code_train
#    print ""
    if not sentence_to_meaning:
        print "*** Generating meaning for test set ... ***"
        (stim_mean_test, l_meaning_code_test) = common.generate_meaning_stim(l_data=test_meaning, l_ocw_array=l_ocw_array_test, full_time=stim_sent_train[0].shape[0], l_m_elt=l_m_elt, verbose=False)
        print "*** ... meaning generated for test set ***"
        print ""
        
    ## Defining reservoir, readout and flow
    reservoir = Oger.nodes.LeakyReservoirNode(output_dim = N, spectral_radius = sr, input_scaling =iss, nonlin_func = np.tanh, leak_rate = leak)
    read_out = mdp.nodes.LinearRegressionNode(use_pinv=True, with_bias=True)
    flow = mdp.Flow([reservoir, read_out])
    if keep_internal_states:
        Oger.utils.make_inspectable(mdp.Flow)
        
    ## Trainning and testing
    print "Train and test"
    if not fast:
        ## test set = train set
        if sentence_to_meaning:
            (states_out_train, internal_states_train, internal_outputs_train, neuron_states_train) = \
                common._teach_and_test_flow(inputs_train_set=stim_sent_train, teacher_outputs_train_set=stim_mean_train, inputs_test_set=stim_sent_train, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)
        else:
            (states_out_train, internal_states_train, internal_outputs_train, neuron_states_train) = \
                common._teach_and_test_flow(inputs_train_set=stim_mean_train, teacher_outputs_train_set=stim_sent_train, inputs_test_set=stim_mean_train, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)
    else:
        raise Exception, "have to define what to do for fast mode"
    ## test set not train set
    if sentence_to_meaning:
        (states_out_test, internal_states_test, internal_outputs_test, neuron_states_test) = \
            common._test_flow(inputs_test_set=stim_sent_test, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)
    else:
        (states_out_test, internal_states_test, internal_outputs_test, neuron_states_test) = \
            common._test_flow(inputs_test_set=stim_mean_test, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)

    if verbose:
        for i in range(len(stim_mean_train)):
            print "len(stim_mean_train)", len(stim_mean_train)
            print "len(l_meaning_code_train)", len(l_meaning_code_train)
            print l_meaning_code_train[i]
            print (stim_mean_train[0]==stim_mean_train[i])
            print (l_meaning_code_train[0]==l_meaning_code_train[i])
    
    ## Writting output meaning
    if sentence_to_meaning:
        l_recovered_meaning_test = convert_l_output_activity_in_meaning(l_out_act=states_out_test, l_ocw_array=l_ocw_array_test, l_m_elt=l_m_elt)
        if verbose:
            print "l_recovered_meaning_test", l_recovered_meaning_test
        l_final_mean_test = []
        for meanings in l_recovered_meaning_test:
            current_meanings = ""
            if verbose:
                print "meanings", meanings
            for i_m in range(len(meanings)):
#                if verbose:
#                    print " i_m:",i_m
#                    print " meanings[i_m]:",meanings[i_m]
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
        
    
    ## Writting output sentence
    if not sentence_to_meaning:
        if not fast and (verbose or plot):
            print ""
            print "**********************************************"
            print "*** Processing recovery of train sentences ..."
            l_recovered_construction_train = convert_l_output_activity_in_construction(l_out_act=states_out_train,
                                                                                       construction_words=construction_words,
                                                                                       min_nr_of_val_upper_thres=1)
            l_recovered_sentences_train = attribute_ocw_to_constructions(l_constructions=l_recovered_construction_train,
                                                                         l_ocw_array=l_ocw_array_train, _OCW='X')
            print "*** l_recovered_sentences_train: ***"
            for s in l_recovered_sentences_train:
                print s
            print "**********************************************"
        if verbose:
            print ""
            print "**********************************************"
            print "*** Processing recovery of test sentences ..."
        l_recovered_construction_test = convert_l_output_activity_in_construction(l_out_act=states_out_test,
                                                                                  construction_words=construction_words,
                                                                                  min_nr_of_val_upper_thres=2)
        l_recovered_sentences_test = attribute_ocw_to_constructions(l_constructions=l_recovered_construction_test,
                                                                    l_ocw_array=l_ocw_array_test, _OCW='X')
        if verbose:
            print "*** l_recovered_sentences_test: ***"
            for s in l_recovered_sentences_test:
                print s
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
        if sentence_to_meaning:
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
        else:
            plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_train_recov", array_=states_out_train, titles_subset=l_recovered_sentences_train, legend_=l_m_elt, plot_slice=None, title="", subtitle="")
            plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_test", array_=states_out_test, titles_subset=l_recovered_sentences_test, legend_=construction_words, plot_slice=None, title="", subtitle="")
    
    
        ## Plot internal states
        if sentence_to_meaning:
            plotting.plot_array_in_file(root_file_name="../../RES_TEMP/intern_states_test", array_=internal_states_test, titles_subset=None, plot_slice=None, title="", subtitle="")
        else:
            plotting.plot_array_in_file(root_file_name="../../RES_TEMP/intern_states_test", array_=internal_states_test, titles_subset=l_ocw_array_test, plot_slice=None, title="", subtitle="")
        print " *** ... Plotting to output file done *** "
        print "**********************************************"
        return l_final_mean_test 

if __name__ == '__main__':
    # global variables
    max_nr_ocw = 8
    max_nr_actionrelation = 2
    elt_pred=['P','F','O','R']
    
    main(path_file_in="../../../AP_input_S.txt",
             path_file_out="../../../AP_output_M.txt")
    print "*********END OF PROGRAM********"
