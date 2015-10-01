# -*- coding: utf-8 -*-
"""
Created on 12 janv. 2012

@author: Xavier HINAUT
xavier.hinaut #/at\# inserm.fr

"""

import mdp
import Oger
import numpy as np

def write_list_in_file(l, file=None, file_path=None):
        """
        Write a list in a file with with one item per line (like a one column csv).
        
        If file is given, then it assumes the file is already open for writing.
        If file_path is given, then it opens the file for writing, write the list, and then close the file.
        """
        if file_path is not None:
            if file is not None:
                raise Exception, "Too much arguments. You must choose between file and file_path."
            else:
                file = open(file_path, 'w')
        if file is None:
            raise Exception, "No file given in input."
        
        for item in l:
            file.write("%s\n" % item)
            
        if file_path is not None:
            file.close()

def extr_sent(sent):
    sent = sent.strip()
    if len(sent)==0:
        raise Exception, " No words in sentence."
    return sent.split(' ')

### Anne changes
def extr_meaning(meaning, verbose=False):
    m_res = []
    a_res = []
    meaning = meaning.strip()
    meaning=meaning.split("<o>")[:-1]

    assignement = meaning[1]
    meaning = meaning[0]

    meaning = meaning.strip()
    
    if ',' in meaning:
        posPAOR = [i for i, letter in enumerate(meaning) if letter == ',']
        nbPAOR = len(posPAOR)
        a = {}
        k = 0
        while k <= nbPAOR:
            key = "m" + k.__str__()
            if k==0:
                a[key] = meaning[0:posPAOR[k]]
            elif k==nbPAOR:
                a[key] = meaning[posPAOR[k-1]+1:len(meaning)]
            else:
                a[key] = meaning[posPAOR[k-1]+1:posPAOR[k]]
            m_res.append(a[key].split())
            k += 1
    else:
        m_res.append(meaning.split())
        
    if ']' in assignement:
        posPAOR = [i for i, letter in enumerate(assignement) if letter == ']']
        nbPAOR = len(posPAOR)
        a = {}
        k = 0
        while k < nbPAOR:
            key = "m" + k.__str__()
            if k==0:
                a[key] = assignement[2:posPAOR[k]]
            elif k==nbPAOR:
                a[key] = assignement[posPAOR[k-1]+2:len(assignement)]
            else:
                a[key] = assignement[posPAOR[k-1]+2:posPAOR[k]]
            m_res.append(a[key].split('-'))
            k += 1
    #else:
    #    m_res.append(assignement.split())    
    #strip(" [],").split('-')
    a_res.append(assignement)
    return m_res
            
def extract_line_train(l):
    """
    Returns a tuple (sentence, meanings):
        sentence is a list of words
        meanings is a list of meaning: [[focus relation obj]] or [[focus relation obj], [focus relation obj]]
            meaning is a list of relations: [focus relation obj] or [focus relation obj1 obj2]
    """
    # ex: guitar over violin;the guitar over the violin is
    (meaning, x, sentence) = l.partition(';')
    
    # processing meaning
    # ex: guitar over violin
    m_res = extr_meaning(meaning, verbose=False)
    
    # processing sentence
    # ex: the guitar over the violin is
    s_res = extr_sent(sent=sentence)
    
    return (s_res, m_res)

def extract_line_test(l):
    print " extracting 1 line of test"
    if ';' in l:
        print "current line:", l
        raise Exception, "Ambiguous line: there should be no ';' because it is a line in <test> ... </test>"
    return extr_sent(sent=l)


def extract_data_io(path_file, verbose=False):
    flag_train = False
    flag_test = False
    
    train = []
    test = []
    sent_form_info_test = []
    
    f = open(path_file, "r")
    for line in f:
            if verbose:
#                print " "
                print "* current line:", line
            #remove commentaries
            (line_tmp,x,x)=line.partition('#')
            # remove useless spaces
            line_tmp = line_tmp.strip()
            if verbose:
                print " after removed commentaries and spaces:", line_tmp
            if line_tmp=='':
                pass
            elif line_tmp[:12]=='<train data>':
                if flag_test == True:
                    raise Exception, "Entering <train data>, but <test data> is not finished. Line concerned: /n"+line
                flag_train = True
            elif line_tmp[:13]=='</train data>':
                if flag_test == True:
                    raise Exception, "Found </train data>, but <test data> is not finished. Line concerned: /n"+line
                flag_train = False
            elif line_tmp[:11]=='<test data>':
                if flag_train == True:
                    raise Exception, "Entering <test data>, but <train data> is not finished. Line concerned: /n"+line
                flag_test = True
            elif line_tmp[:12]=='</test data>':
                if flag_train == True:
                    raise Exception, "Found </test data>, but <train data> is not finished. Line concerned: /n"+line
                flag_test = False
            else:
				# in train mode
                if flag_train:
                    x = extract_line_train(l=line_tmp)
                    train.append(x)
					
                elif flag_test:
                    y = extract_line_test(l=line_tmp)
                    test.append(y)
                    canonical_info = None
                    sent_form_info_test.append(canonical_info)
    f.close()
    return [train, test, sent_form_info_test]

def txt2corpus_and_meaning(train_txt):
    train_corpus, train_meaning  = [], []
    ## For each (sentence,meanings) tuple
    for (s,m) in train_txt:
        train_corpus.append(s)
        train_meaning.append(m)
    return train_corpus, train_meaning

def extrac_open_class_words(l_sent, _OCW, l_closed_class):
    """
    make a new sentence (list of words). words that are not in get_closed_class_words() are changed in _OCW (Open Class Word)
    
    Inputs:
        l_sent: list of words: sentence is in the form of a list of words
        _OCW (Open Class Word): string that will replace open class words in the new formed sentence
            if the string is equal to '_', then the open class words will be replaced by a pause in the stimulus
            (i.e. there won't be a specific input for open class words, there will just be no stimulus during the same amount of time of a normal word)
        l_closed_class: list of closed class words
        
    Outputs:
        ocw_array: open class word array (cf. Dominey, Hoen, Inui 2006. A neurolinguistic model of grammatical construction processing. JoCN 18:12, pp.2088-2107.
        l_sent_ccw: new sentence with open class words replaced by _OCW
    
    """
    ocw_array, l_sent_ccw = [], []
    for w in l_sent:
        if w in l_closed_class:
            l_sent_ccw.append(w)
        else:
            l_sent_ccw.append(_OCW)
            ocw_array.append(w)
    return (ocw_array, l_sent_ccw)

def get_and_remove_ocw_in_corpus(corpus, _OCW, l_closed_class):
    new_corpus = []
    l_ocw_array = []
    for s in corpus:
        (ocw_array, l_sent_ccw) = extrac_open_class_words(l_sent=s, _OCW=_OCW, l_closed_class=l_closed_class)
        new_corpus.append(l_sent_ccw)
        l_ocw_array.append(ocw_array)
    if _OCW=='_':
        construction_words = l_closed_class
    else:
        construction_words = l_closed_class+[_OCW]
    return (new_corpus, l_ocw_array, construction_words)

def get_meaning_coding(max_nr_ocw=10, max_nr_actionrelation=3, elt_pred=['P','A','O','R','V']):
    """
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
    l = []
    for i in range(1,max_nr_ocw+1):
        for j in range(1,max_nr_actionrelation+1):
            for elt_p in elt_pred:
                l.append('_'+str(i)+'-'+str(elt_p)+str(j))
    return l
#    return (l, max_nr_ocw, max_nr_actionrelation, elt_pred)

def is_there_several_time_the_same_elt_in_list(l):
    """
    Returns False if each element of the list is unique
    Otherwise, returns a list of elements which have several occurrences
    """
    l_res = []
    for elt in l:
        if l.count(elt)>1:
            l_res.append(elt)
    if len(l_res)==0:
        return False
    else:
        return l_res


def generate_meaning_stim(l_data, l_ocw_array, full_time, l_offset=None, l_m_elt=None,
                          elt_pred=['P','A','O','R','V'], ocw_predicate_matches_with_one_meaning=False,
                          initial_pause=True, pause=True, act_time=None, verbose=False):
    """ 
    
    Inputs:
        - 
        - 
        - l_offset: The offset represents the difference between the maximum number of words in the data and the number of word of a given sentence.
        - 
        - ocw_predicate_matches_with_one_meaning: each predicate should match with only one meaning
            e.g. in the meanings [['left', 'trumpet', 'violin'], ['left', 'trumpet', 'guitar']]
                -> 'left' in meaning 1 is not the same 'left' than in meaning 2
        
        
    ---Old description---
    Gives the hole teacher signal data set 'English Grammatical Construction' or a subset of the data set defined by the list 'subset' in dictionary dp.
    The sentences are aligned to the right in this version '_offset_end'
    
    Modification of method get_teacher_output(act_time, pause, suppl_pause_at_the_end, subset=None):
        in order to be able to set an arbitrary moment where to begin the output.
        if 'start' is set to 'end', this means that the output will be asked since the beginning of the last "element" of the sentence (an element could be a word or a sign of punctuation like a dot).
    
    Inputs:
        - start: if it's a number it indicates the position of the word where will start the teacher output signal (this number starts from 1, not from 0)
            (see method _output_gen_start_act_time() for further details)
        - initial_pause indicates if the stimulus begins with a pause (during a time corresponding to 'act_time')
    """
    def make_stim():
        m_code = '_'+str(idx_ocw+1)+'-'+str(elt_p)+str(idx_m+1)
        if verbose:
            print "   m_code", m_code
        idx = l_m_elt.index(m_code)
        indices_m_code.append(idx)
        if verbose:        
            print "   idx", idx
        if l_offset is None:
            stim_seq[:full_time,idx] = np.ones((full_time,1)).T
        else:
            st = l_start_TS[seq_id]
            stim_seq[:st,idx] = np.zeros((st,1)).T
            stim_seq[st:full_time,idx] = np.ones((full_time-st,1)).T
        meaning_code.append(m_code)
    
    def get_starting_point_in_act_time(offset):
        """
        starting point in act_time (Activation Time)
        """
        return initial_pause*1 + offset*(1*(pause==False)+2*(pause==True))
    
    print "** generate_meaning_stim ..."
    
    
#    if subset is None:
#        subset = range(len(l_data))
    if l_m_elt is None:
        l_m_elt = get_meaning_coding()

    if l_offset is not None:
        if act_time is None:
            raise Exception, "Could not use offset information to generate data, act_time information is compulsory to use the offset."
        l_start_TS = [] # start in time step
        for i_os in range(len(l_offset)):
            start_AT = get_starting_point_in_act_time(l_offset[i_os])
            l_start_TS.append(start_AT*act_time)
        
        

#    stim = len(l_data)*[np.zeros((len(l_m_elt), full_time))]
#    stim = len(l_data)*[np.zeros((full_time,len(l_m_elt)))]
    stim = []
    l_meaning_code = []
    l_indices_m_code = []
    for seq_id in range(len(l_data)):
        # For each sentence
#        print " *** seq_id "+str(seq_id)+" ... ***"
        meanings = l_data[seq_id]
        ocw_array = l_ocw_array[seq_id]
        stim_seq = np.zeros((full_time,len(l_m_elt)))
        meaning_code = []
        indices_m_code = []
#        print " meanings", meanings
#        print " ocw_array", ocw_array
        if is_there_several_time_the_same_elt_in_list(l=ocw_array):
            if verbose:
                print "  ! There is several times the same OCW(s) in the current ocw_array."
                print "  ! This(These) Open Class Words is(are):"+str(is_there_several_time_the_same_elt_in_list(l=ocw_array))
        ## TODO: it may be more effecient reversing the order of the "for" loops (interverting idx_m with idx_ocw)
        ## New-way: for loop on OCW, then for loop on meaning
        #For each Open Class Word in ocw_array
        dic_predicate_already_attributed = {} # dictionary of predicates already attributed
        for idx_ocw in range(len(ocw_array)):
            # For each open class word in ocw_array
#            print "  idx_ocw", idx_ocw
#            print "  l_ocw_array[idx_ocw]", ocw_array[idx_ocw]
            for idx_m in range(len(meanings)):
                # For each meaning
#                print "   idx_m", idx_m
#                print "   meanings[idx_m]", meanings[idx_m]
                if ocw_array[idx_ocw] in meanings[idx_m]:
#                    print "   meanings[idx_m].index(ocw_array[idx_ocw])", meanings[idx_m].index(ocw_array[idx_ocw])
                    i_action = meanings[idx_m].index(ocw_array[idx_ocw])
                    elt_p = elt_pred[i_action]
#                    print "   elt_p", elt_p
                    # if element is a predicate and if each predicate should match with only one meaning
                    if elt_p=='P' and ocw_predicate_matches_with_one_meaning:
                        # if the predicate has already been coded/attributed
                        if (idx_ocw in dic_predicate_already_attributed.keys()) or (idx_m in dic_predicate_already_attributed.values()):
                            if verbose:
                                print "   !predicate "+str(ocw_array[idx_ocw])+" has already been attributed to meaning "+str(meanings[idx_m])
                                print "     dic_predicate_already_attributed:"+str(dic_predicate_already_attributed)
                        else:
                            # add this predicate with meaning to which at has been attributed
#                            dic_predicate_already_attributed.append(idx_ocw)
                            dic_predicate_already_attributed[idx_ocw] = idx_m
                            make_stim()
                    else:
                        make_stim()
                #else:
                #    print "    -X- "+str(ocw_array[idx_ocw])+" not in "+str(meanings[idx_m])
#            print "  indices_m_code:",indices_m_code
        stim.append(stim_seq)
        l_meaning_code.append(meaning_code)
        l_indices_m_code.append(indices_m_code)
    
    print "** ... generate_meaning_stim"
    return (stim, l_meaning_code)

    
def _teach_flow(inputs, teacher_outputs, _flow, _reservoir=None, reset_state_before_begin=None):
    """
    TODO: see if the reset of the states (when reset_state_before_begin is true) if working
    """
    if reset_state_before_begin==True:
        _reservoir.initial_states = mdp.numx.zeros((1, _reservoir.output_dim))
    learning_data = [inputs, zip(inputs, teacher_outputs)]
    _flow.train(learning_data)
    return _flow

def _test_flow(inputs_test_set, _flow, _reservoir, keep_internal_states=False, type_reservoir="", reset_state_before_begin=None):
    """
    TODO: change the use of type_reservoir, used _reservoir instead
    """
    states_out = len(inputs_test_set)*[None]
    internal_states = []
    internal_outputs = []
    neuron_states = []
    if keep_internal_states==True:
        if type_reservoir == "LeakyReservoirNode" or (type(_reservoir) is Oger.nodes.LeakyReservoirNode):
            internal_states = len(inputs_test_set)*[None]
        elif type_reservoir == "LeakyNSLNodeOutputs" or type_reservoir == "LeakyNSLNodeOutputsNoise":
            internal_outputs = len(inputs_test_set)*[None]
            neuron_states = len(inputs_test_set)*[None]
        else:
            #raise exception
            print "No reservoir type given in _test_flow()"
    # Testing the flow for each input in the test set
    for idx_out in range(len(inputs_test_set)):
        if idx_out==0 and reset_state_before_begin==True:
#            _reservoir.initial_states = mdp.numx.zeros((1, _reservoir.output_dim))
            _reservoir.reset_states = True
        states_out[idx_out] = _flow(inputs_test_set[idx_out])
        if idx_out==0 and reset_state_before_begin==True:
            _reservoir.reset_states = False
        if keep_internal_states==True:
            if type_reservoir == "LeakyReservoirNode" or (type(_reservoir) is Oger.nodes.LeakyReservoirNode):
                #saving internal states of LeakyReservoirNode neurons
                internal_states[idx_out] = _flow.inspect(_reservoir)
            elif type_reservoir == "LeakyNSLNodeOutputs" or type_reservoir == "LeakyNSLNodeOutputsNoise":
                # Saving for internal outputs of LeakyNSLNodeOutputs
                internal_outputs[idx_out] = _flow.inspect(_reservoir)
                # Saving for internal states of LeakyNSLNodeOutputs
                neuron_states[idx_out] = _reservoir.get_states() #neuron_ouputs_temp = reservoir.get_outputs() 
    return (states_out, internal_states, internal_outputs, neuron_states)

def _teach_and_test_flow(inputs_train_set, teacher_outputs_train_set, inputs_test_set, _flow, _reservoir, keep_internal_states=False, type_reservoir="", return_flow=False, reset_state_before_begin=None):
    """
    Friday 4th February, 2011
    New method that is using both method to teach and train the flow separately. It replaces the method '_teach_and_train_flow' (now renamed '_teach_and_test_flow_old'').
    """
    flow_trained = _teach_flow(inputs=inputs_train_set, teacher_outputs=teacher_outputs_train_set, _flow=_flow, _reservoir=_reservoir, reset_state_before_begin=reset_state_before_begin)
    (states_out, internal_states, internal_outputs, neuron_states) = _test_flow(inputs_test_set=inputs_test_set, _flow=flow_trained, _reservoir=_reservoir, keep_internal_states=keep_internal_states, type_reservoir=type_reservoir, reset_state_before_begin=reset_state_before_begin)
    if return_flow:
        return (states_out, internal_states, internal_outputs, neuron_states, flow_trained)
    else:
        return (states_out, internal_states, internal_outputs, neuron_states)

