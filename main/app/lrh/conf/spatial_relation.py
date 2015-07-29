# -*- coding: utf-8 -*-
"""
Created on 11 janv. 2012

@author: Xavier HINAUT
xavier.hinaut #/at\# inserm.fr
"""

import mdp
import numpy as np
import reservoir
    
### Extraction Methods ###
##########################
    


def get_closed_class_words():
    """
    list of closed class words (the correct list depends on the corpus)
    """
    return ['after', 'than', 'is', 'of', 'and', 'before', 'to', 'the', 'slowly', 'quickly', 'was', 'with', 'that', 'for', 'a', 'now'];
    #return ['is', 'of', 'the', 'to']
    #return closed_class_wordsSD

def extr_sent(sent):
    sent = sent.strip() #removing spaces before and after the sentence
    if len(sent)==0:
        raise Exception, "No words in sentence."
    return sent.split(' ')

def extr_meaning(meaning, verbose=False):
    #print "    current meaning: "+str(meaning)
    m_res = []
    a_res = []
    meaning = meaning.strip()
    meaning=meaning.split("<o>")[:-1]
    (m1, x, m2) = meaning[0].partition(',')
    assignement = meaning[1]
    m1 = m1.strip()
    m2 = m2.strip()
    assignement = assignement.strip()
    if len(m1)<3 or len(m1)>4:
        m_res.append(m1.split())
    else:
        raise Exception, "Number of words not good for 1st meaning: "+str(len(m1))
    if m2!='':
        if len(m2)<3 or len(m2)>4:
            m_res.append(m2.split())
        else:
            raise Exception, "Number of words not good for 1st meaning: "+str(len(m1))
    #print "    result extr meaning: "+str(m_res)
    a_res.append(assignement)
    return m_res, a_res

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
    m_res, a_res = extr_meaning(meaning, verbose=True)
    
    # processing sentence
    # ex: the guitar over the violin is
    s_res = extr_sent(sent=sentence)
    
    return (s_res, m_res, a_res)
    
def extract_line_test(l, sentence_to_meaning=False):
    if ';' in l:
        #print "current line:", l
        raise Exception, "Ambiguous line: there should be no ';' because it is a line in <test> ... </test>"
    if sentence_to_meaning:
        return extr_sent(sent=l)
    else:
        if "<o>" in l:
            l = l.strip()
            #print "  m:["+str(l)+"]"
            sentence, assignement = extr_meaning(meaning=l, verbose=True)
            return (sentence, assignement) #assignement is currently replacing canonical information
        else:
            raise Exception, "APOR structure missing"

def extract_data_io(path_file, sentence_to_meaning=False, verbose=True):
    flag_train = False
    flag_test = False
    
    train = []
    test = []
    sent_form_info_train = []
    sent_form_info_test = []
    
    f = open(path_file, "r")
    corpus=f.readlines()
    
    for line in corpus:
            #if verbose:
                #print " "
                #print "* current line:", line
            #remove commentaries
            (line_tmp,x,x)=line.partition('#')
            # remove useless spaces
            line_tmp = line_tmp.strip()
            #if verbose:
                #print "after removed commentaries and spaces:", line_tmp
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
                if flag_train:
                    x = extract_line_train(l=line_tmp)
                    train.append(x[0:2])
                    sent_form_info_train.append(x[2][0])
                elif flag_test:
                    y = extract_line_test(l=line_tmp, sentence_to_meaning=sentence_to_meaning)

                    if type(y) is tuple:
                        assignement = y[1][0]
                        meaning = y[0]
                    test.append(meaning)
                    sent_form_info_test.append(assignement)
    f.close()
    return [train, test, sent_form_info_train, sent_form_info_test]


### Sentence & Meanning Methods ###
###################################

def txt2corpus_and_meaning(train_txt):
    train_corpus, train_meaning  = [], []
    ## For each (sentence,meanings) tuple
    for (s,m) in train_txt:
        train_corpus.append(s)
        train_meaning.append(m)
    return train_corpus, train_meaning
    

def extrac_open_class_words(l_sent, _OCW, l_closed_class=get_closed_class_words()):
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

#COLAS : modified function for the recoding of the meaning
def get_and_remove_ocw_in_corpus(corpus, _OCW):
    new_corpus = []
    for s in corpus:
        (ocw_array, l_sent_ccw) = extrac_open_class_words(l_sent=s, _OCW=_OCW, l_closed_class=get_closed_class_words())
        new_corpus.append(l_sent_ccw)
    if _OCW=='_':
        construction_words = get_closed_class_words()
    else:
        construction_words = get_closed_class_words()+[_OCW]
    return (new_corpus, construction_words)

def is_nr_ocw_in_construction_ok(construction, ocw_array, _OCW):
    """
    Checks if the number of OCW in ocw_array corresponds to the number of _OCW in the construction.
    If the number of OCW in construction is OK, then it returns True.
    """
    if construction.count(_OCW) != len(ocw_array):
        return False
    else:
        return True

def attribute_ocw_to_constructions(l_constructions, l_ocw_array, _OCW):
    #print "---*** attribute_ocw_to_constructions ..."
    l_sent = []
    for idx_c in range(len(l_constructions)):
        sent = []
        ocw_arr = list(l_ocw_array[idx_c])
        if not is_nr_ocw_in_construction_ok(construction=l_constructions[idx_c], ocw_array=ocw_arr, _OCW=_OCW):
            diff = l_constructions[idx_c].count(_OCW) - len(ocw_arr)
            if diff>0:
                # if the number of OCW in ocw_array is not enough, add one
                empty_list = diff*['_X_']
                ocw_arr.extend(empty_list)
        ocw_arr.reverse()
        for word in l_constructions[idx_c]:
            if word==_OCW:
                w = ocw_arr.pop()
                sent.append(w)
            else:
                sent.append(word)
        l_sent.append(sent)
    return l_sent


#COLAS: strucure_partition() and generate_l_ocw_array() replace the function generate_l_ocw_array_in_PAOR_order
def structure_partition(structure):
    structure1, x, structure2 = structure.partition(']')
    structure1=structure1.strip(" [],").split('-')
    structure2=structure2.strip(" [],").split('-')

    return structure1, structure2

def generate_l_ocw_array(tab_structure, tab_meaning):
    l_ocw_array=[]
    for structure, meaning in zip(tab_structure, tab_meaning):
        ocw_array=[]
        #meaning extraction for each sentence
        meaning1=meaning[0]
        if len(meaning)>1:  meaning2=meaning[1]
        #structure extraction for each sentence
        structure1, structure2 = structure_partition(structure)
        #correspondance between words of the meaning and structure
        cor={'P':0, 'A':1, 'O':2, 'R':3, 'X':4}
        if len(structure2)==1:
            for role1 in structure1:
                if role1!='_':  ocw_array.append(meaning1[cor[role1]])
        else:
            for role1, role2 in zip(structure1, structure2):
                if role1!='_':  ocw_array.append(meaning1[cor[role1]])
                elif role2!='_':   ocw_array.append(meaning2[cor[role2]])
        l_ocw_array.append(ocw_array)
    return l_ocw_array



def get_meaning_coding(max_nr_ocw=14, max_nr_actionrelation=4, elt_pred=['P','A','O','R']):
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

#COLAS : function completely recoded
def generate_meaning_stim(l_structure, full_time, l_m_elt):

    l_meaning_code = []
    l_indices_m_code = []
    stim = []
    for structure in l_structure:
        stim_seq = np.zeros((full_time,len(l_m_elt)))
        indices_m_code=[]
        meaning_code=[]
        structure1, structure2 = structure_partition(structure)
        if len(structure2)==1: #if structure2 is empty, i.e simple sentence
            for pos,role1 in enumerate(structure1):
                if role1!='_':
                    m_code = '_'+str(pos+1)+'-'+role1+'1'
                    idx = l_m_elt.index(m_code)
                    indices_m_code.append(idx)
                    stim_seq[:full_time,idx] = np.ones((full_time,1)).T
                    meaning_code.append(m_code)
        else: #if complex sentence
            for pos,role1, role2 in zip(range(len(structure1)), structure1, structure2):        
                if role1!='_':
                    m_code = '_'+str(pos+1)+'-'+role1+'1'
                    idx = l_m_elt.index(m_code)
                    indices_m_code.append(idx)
                    stim_seq[:full_time,idx] = np.ones((full_time,1)).T
                    meaning_code.append(m_code)
                if role2!='_':
                    m_code = '_'+str(pos+1)+'-'+role2+'2'
                    idx = l_m_elt.index(m_code)
                    indices_m_code.append(idx)
                    stim_seq[:full_time,idx] = np.ones((full_time,1)).T
                    meaning_code.append(m_code)
        stim.append(stim_seq)
        l_meaning_code.append(meaning_code)
        l_indices_m_code.append(indices_m_code)
    return (stim, l_meaning_code)


### Teaching and testing methods ###
####################################
#COLAS : all the teaching and testing function have been deleted. The didn't serve anymore thanks to the new reservoir.

def convert_output_activity_in_signal_idx_max(out_act,  thres, eps):
    # Each vector should be like this : stim_seq = np.zeros((full_time,len(construction_words)))
    signal_indices_max = []
    # Threshold array
    out_act = out_act*(out_act>thres)
    # for each time step
#    for arr in len(out_act.shape[10]):
    for i in range(out_act.shape[0]):
        arr = out_act[i]
        # look which output activity signal is maximum
        maxi = np.max(arr)
        if maxi<eps:
            # The max is 0, so no signal is upper than the threshold
            idx=-1
        else:
            idx = list(arr).index(maxi)
            tmp = list(arr)
            tmp.remove(maxi)
            if max(tmp)==maxi:
                # there is at least 2 values that are equal to maximum
                idx = -2
        signal_indices_max.append(idx)
    return signal_indices_max


#COLAS: function slightly modified to solve the bug happenning when 2 open class words are juxtaposed
def convert_one_output_activity_in_construction(out_act, construction_words, min_nr_of_val_upper_thres=1):
    """
    Inputs:
        - min_nr_of_val_upper_thres : number of values upper the threshold needed to take into account the word
            The default value is 1: this indicates that this parameters is useless,
                because 1 occurrence of an index is enough to add the corresponding word in the sentence.
            For instance if min_nr_of_val_upper_thres equals 2, it will not take into account singular
                pics into account.
    
    """
    # Each vector should be like this : stim_seq = np.zeros((full_time,len(construction_words)))
    
    signal_indices_max = convert_output_activity_in_signal_idx_max(out_act,  thres=0.4, eps = 1e-12)
    #print "signal_indices_max:", signal_indices_max
    previous = -1
    keep_in_merory = -1
    nr_occurrence_same_index = 0
    sent = []
    for i in range(len(signal_indices_max)):
        if signal_indices_max[i]!=previous:
            # if the new signal was the same that the one kept in memory
            if signal_indices_max[i]==keep_in_merory:
                # increment the counter nr_occurrence_same_index
                nr_occurrence_same_index = nr_occurrence_same_index + 1
                #if signal_indices_max[i]!=-1:
                    #print "keep in memory this index: ", signal_indices_max[i]
                    #print " - nr_occurrence_same_index: ", nr_occurrence_same_index
            # if we have to wait for more occurrences of this index to take it into account
            if (min_nr_of_val_upper_thres-1-nr_occurrence_same_index) > 0:
                # keep the index in memory
                #if signal_indices_max[i]!=-1:
                    #print " - still have to wait"
                keep_in_merory = signal_indices_max[i]
            else:
                # add the word corresponding to this index in the final sentence
                if signal_indices_max[i]!=-1:
                    ##print "new idx detected:", signal_indices_max[i]
                    word = construction_words[signal_indices_max[i]]
                    #print "corresponding word:", word         
                    sent.append(word)
                previous = signal_indices_max[i]
                # reinitialize temporary variables
                nr_occurrence_same_index = 0
                keep_in_merory = -1  
    #if sent==[]:
        #raise Exception, "No words has been generated by the network. Output activity may be too low."
    return sent
    
def convert_l_output_activity_in_construction(l_out_act, construction_words, min_nr_of_val_upper_thres=1):
    l_sent = [] # list of list of words i.e. list of sentences
    sentence_nb=1
    for out_act in l_out_act:
        #print "training sentence number :", sentence_nb
        sent = convert_one_output_activity_in_construction(out_act, construction_words, min_nr_of_val_upper_thres=min_nr_of_val_upper_thres)
        l_sent.append(sent)
        sentence_nb+=1
    return l_sent

#COLAS: treshold function to prevent the divergence of the reservoir (used with feedback)
def treshold_signal(vect, sup, inf):
    for i in range(len(vect)):
        if vect[i]>=sup:    vect[i]=sup
        elif vect[i]<=inf:  vect[i]=inf
    return vect


### Main Methods ###
##########################
#COLAS : function modified in order to give parameters in argument. New reservoir as well. Feedback implemented but it doesn't work for now (see report and readme)
def main(path_file_in, path_file_out,N=400, sr=3, iss=0.1, leak=0.1, ridge=10**-1, plot=False, feedback=False, return_result=False, verbose=False):
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

    import io_language_coding as CtIolangcod
    sentence_to_meaning = False
    
    # Definning parameters of stimulus (in a dictionary)
    d = {}
    d['act_time'] = 5
    d['pause'] = True
    d['suppl_pause_at_the_end'] = 1*d['act_time']
    d['initial_pause'] = True
    d['offset'] = False
    
    
    ## Random parameters
    import time
    millis = int(round(time.time() ))    
    seed = millis#2#4#2

    if seed is not None:
        mdp.numx.random.seed(seed)
        np.random.seed(seed)
        
    [train_data_txt, test_data_txt, sent_form_info_train, sent_form_info_test] = extract_data_io(path_file=path_file_in, sentence_to_meaning=sentence_to_meaning)

    train_corpus, train_meaning  = txt2corpus_and_meaning(train_txt=train_data_txt)
    if sentence_to_meaning:
        test_corpus = test_data_txt
    else:
        test_meaning = test_data_txt
    # making the list of constructions (refering to "construction grammar"), a construction is a sentence without its open class words (Nouns and Verbs)
    (l_construction_train, construction_words) = get_and_remove_ocw_in_corpus(corpus=train_corpus, _OCW='X')
    l_ocw_array_train=generate_l_ocw_array(sent_form_info_train, train_meaning)
    l_ocw_array_test=generate_l_ocw_array(sent_form_info_test, test_meaning)
    #print "**************************"
    #print "l_construction_train", l_construction_train
    #print "construction words", construction_words
    if sentence_to_meaning:
        (l_construction_test, construction_words_test) = get_and_remove_ocw_in_corpus(corpus=test_corpus, _OCW='X')
        #print "l_construction_test", l_construction_test
        if construction_words!=construction_words_test:
            raise Exception, "The construction words are not the same for the train constructions and the test constructions. So the coding of sentences will be different and should provoque a future problem."
    
    ## Generating all the sentence stimulus (in order to have the same length for each sentence)
    if sentence_to_meaning:
        ## Generate the stimulus input for train and test data
        l_full_const = l_construction_train + l_construction_test
        slice_test = slice(len(l_construction_train),len(l_construction_train)+len(l_construction_test))

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

    
    l_m_elt = get_meaning_coding()

    (stim_mean_train, l_meaning_code_train) = generate_meaning_stim(l_structure=sent_form_info_train, full_time=stim_sent_train[0].shape[0], l_m_elt=l_m_elt)

    if not sentence_to_meaning:
        #print "*** Generating meaning for test set ... ***"
        (stim_mean_test, l_meaning_code_test) = generate_meaning_stim(l_structure=sent_form_info_test, full_time=stim_sent_train[0].shape[0], l_m_elt=l_m_elt)

    other_corpus_used = False

    # Reservoir and Read-out definitions
    res = reservoir.Reservoir(N, sr, iss, leak)

    #classic working of the reservoir
    if feedback==False:
        ## test set = train set
        states_out_train, internal_states_train = res.train (stim_mean_train, stim_sent_train)
        ## test set not train set
        states_out_test, internal_states_test = res.test(stim_mean_test)
    #feedback working of the reservoir. !! Should be implemented directly in the reservoir class !!
    else:
        delay=1
        nb_epoch_max=4
        dim_input = stim_mean_train[0].shape[1]
        dim_output =  len(stim_sent_train[0][0])
        input_train=[]

        for (x,y) in zip( np.copy(stim_mean_train), np.copy(stim_sent_train)):
            for time_step_delay in range(delay):
                y=np.concatenate( ([[0.]*len(y[0])] , y), axis=0)
            input_train.append(np.array(  np.concatenate(   (x, y[:-delay]), axis=1 )  ))
      
        nb_train=0  
        while nb_train < nb_epoch_max:
            ## test set = train set
            states_out_train, internal_states_train = res.train (input_train, stim_sent_train)
            
            tab_feedback=[]
            for num_phrase in range(len(states_out_train)):
                #signal tresholded
                states_out_train[num_phrase]=np.array([treshold_signal(signal_t,1.5,-0.5) for signal_t in states_out_train[num_phrase]])
                if nb_train==0: #feedback kept only for the first train
                    #feedback assignation
                    feedback=np.array(states_out_train[num_phrase])
                    #signal delayed
                    for time_step_delay in range(delay):
                        feedback=np.concatenate( ([[0.]*len(feedback[0])] , feedback), axis=0)
                
                tab_feedback.append(feedback)
                input_train[num_phrase]=input_train[num_phrase].T
                input_train[num_phrase][dim_input:] = feedback[:-delay].T
                input_train[num_phrase]=input_train[num_phrase].T

            nb_train+=1

        ## test set not train set
        for t in range(0,stim_mean_test[0].shape[0],1):
            input_test=[]
            if t==0: #A REMODIFIER
                for n_phrase in range(len(stim_mean_test)):
                    input_test.append(np.concatenate(  (stim_mean_test[n_phrase][t:t+1,:] , [[0.]*len(stim_sent_train[0][0])] ) , axis=1     ) )

                states_out_test, internal_states_test = res.test(input_test)
                import copy
                states_out_test_def=copy.deepcopy(states_out_test)

            else:
                for n_phrase in range(len(stim_mean_test)):
                    #feedback assignation
                    feedback=np.array(states_out_test[n_phrase])
                    input_test.append(np.concatenate(  (stim_mean_test[n_phrase][t:t+1,:] , feedback ) , axis=1     ) )

                states_out_test, internal_states_test = res.test(input_test)
            
                for n_phrase in range(len(stim_mean_test)):
                    states_out_test_def[ n_phrase ]=np.concatenate( (states_out_test_def[n_phrase] , states_out_test[n_phrase]), axis=0  )

        states_out_test=states_out_test_def



    
    # Ecriture de la phrase de réponse
    if other_corpus_used:
        var_inutile=0

    else:

        l_recovered_construction_train = convert_l_output_activity_in_construction(l_out_act=states_out_train,
                                                                                   construction_words=construction_words,
                                                                                   min_nr_of_val_upper_thres=1)
        l_recovered_sentences_train = attribute_ocw_to_constructions(l_constructions=l_recovered_construction_train,
                                                                     l_ocw_array=l_ocw_array_train, _OCW='X')

        l_recovered_construction_test = convert_l_output_activity_in_construction(l_out_act=states_out_test,
                                                                                  construction_words=construction_words,
                                                                                  min_nr_of_val_upper_thres=2)
        l_recovered_sentences_test = attribute_ocw_to_constructions(l_constructions=l_recovered_construction_test,
                                                                    l_ocw_array=l_ocw_array_test, _OCW='X')
    
    
        ## Writting sentences to output file
        #print " *** Writting to output file ... *** "
        l_final_sent_test = []
        for list_words in l_recovered_sentences_test:
            l_final_sent_test.append(" ".join(list_words))

        
        #print " *** ... Writting done ***"
        #print "**********************************************"
        print "********************************************** "
        print " *** RECOGNIZED SENTENCES *** "
        print l_final_sent_test[0]

        write_list_in_file(l=l_final_sent_test, file_path=path_file_out)
        if return_result:   
            return l_final_sent_test

    ## Plot inputs
    if plot:
        import plotting as plotting
    
        plotting.plot_array_in_file(root_file_name="../Results/states_out_train", array_=states_out_train, titles_subset=l_construction_train, legend_=construction_words, plot_slice=None, title="", subtitle="")

        plotting.plot_array_in_file(root_file_name="../Results/states_out_test", array_=states_out_test, titles_subset=l_recovered_sentences_test, legend_=construction_words, plot_slice=None, title="", subtitle="")

    print ""

if __name__ == '__main__':
    import sys
    print "############################################################################################"
    corpusFileSD = sys.argv[1]
    fileResult = sys.argv[2]
        
    closed_class_wordsSD = sys.argv[3].split(',') # ['after', 'and', 'before', 'to', 'the', 'slowly', 'quickly', 'was', 'with', 'that', 'for', 'a', 'now']

    max_nr_ocw = int(sys.argv[4])              #10
    max_nr_actionrelation = int(sys.argv[5])    #4
    elt_pred= sys.argv[6].split(',')     #['P','A','O','R','V']
    #    
    main(path_file_in=corpusFileSD, path_file_out=fileResult, plot=False, feedback=False)


    #corpusFileSD = "/home/anne/.local/share/yarp/contexts/lrh/conf/Corpus/temporaryCorpus.txt"
    #fileResult = "/home/anne/.local/share/yarp/contexts/lrh/conf/Corpus/output.txt"
    #closed_class_wordsSD =  ['after', 'and', 'before', 'to', 'the', 'slowly', 'quickly', 'was', 'with', 'that', 'for', 'a', 'now']
    #print "closed_class_wordsSD ", closed_class_wordsSD   
    #max_nr_ocw = 10
    #max_nr_actionrelation = 4
    #elt_pred = ['P','A','O','R','V']
    print "*********END OF PROGRAM********"
