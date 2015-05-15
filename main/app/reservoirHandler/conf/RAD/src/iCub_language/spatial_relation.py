#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on 11 janv. 2012

@author: Xavier HINAUT
xavier.hinaut #/at\# inserm.fr
"""

import mdp
import Oger
import numpy as np

### Extraction Methods ###
##########################

def extr_sent(sent):
    sent = sent.strip()
    if len(sent)==0:
        raise Exception, "No words in sentence."
    return sent.split(' ')

def extr_meaning(meaning, verbose=False):
    print "    current meaning: "+str(meaning)
    m_res = []
    meaning = meaning.strip()
    (m1, x, m2) = meaning.partition(',')
    m1 = m1.strip()
    m2 = m2.strip()
    if len(m1)<3 or len(m1)>4:
        m_res.append(m1.split())
    else:
        raise Exception, "Number of words not good for 1st meaning: "+str(len(m1))
    if m2!='':
        if len(m2)<3 or len(m2)>4:
            m_res.append(m2.split())
        else:
            raise Exception, "Number of words not good for 1st meaning: "+str(len(m1))
    print "    result extr meaning: "+str(m_res)
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
    m_res = extr_meaning(meaning, verbose=True)
    
    # processing sentence
    # ex: the guitar over the violin is
    s_res = extr_sent(sent=sentence)
    
    return (s_res, m_res)
    
def extract_line_test(l, sentence_to_meaning=False):
    if ';' in l:
        print "current line:", l
        raise Exception, "Ambiguous line: there should be no ';' because it is a line in <test> ... </test>"
    if sentence_to_meaning:
        return extr_sent(sent=l)
    else:
        if ":" in l:
            (m,x,canonical_info)=l.partition(':')
            m = m.strip()
            canonical_info = canonical_info.strip()
            print "  m:["+str(m)+"]"
            print "  canonical_info:["+canonical_info+"]"
            sentence = extr_meaning(meaning=m, verbose=True)
            if canonical_info[0]=='C':
                sent_form_info = 0
            elif canonical_info[0]=='N':
                sent_form_info = 1
            else:
                print "current line:", l
                raise "Exception", "Sentence form information (canonical or non-canonical) not understood."
            return (sentence, sent_form_info)
        else:
            return extr_meaning(meaning=l, verbose=True)

def extract_data_io(path_file, sentence_to_meaning=False, verbose=True):
    flag_train = False
    flag_test = False
    
    train = []
    test = []
    sent_form_info_test = []
    
    f = open(path_file, "r")

    for line in f:
            print "###################################"
            print line
            if verbose:
                print " "
            print "* current line:", line
            #remove commentaries
            (line_tmp,x,x)=line.partition('#')
            print "line_tmp"
            print x
            print line_tmp
            print x
            # remove useless spaces
            if verbose:
                print "after removed commentaries and spaces:", line_tmp
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
                    train.append(x)
                elif flag_test:
                    y = extract_line_test(l=line_tmp, sentence_to_meaning=sentence_to_meaning)
                    if type(y) is list:
                        canonical_info = None
                        meaning = y
                    elif type(y) is tuple:
                        canonical_info = y[1]
                        meaning = y[0]
                    test.append(meaning)
                    sent_form_info_test.append(canonical_info)
    f.close()
#    return [train, test]
    return [train, test, sent_form_info_test]

### Sentence & Meanning Methods ###
###################################

def txt2corpus_and_meaning(train_txt):
    train_corpus, train_meaning  = [], []
    ## For each (sentence,meanings) tuple
    for (s,m) in train_txt:
        train_corpus.append(s)
        train_meaning.append(m)
    return train_corpus, train_meaning

def get_closed_class_words():
    """
    list of closed class words
    """
#    return ['a', 'an', 'and', 'are', 'as', 'at', 'be', 'but', 'by', 'did', 'for', 'from', 'he', 'her', 'had', 'has', 'have', 'him', 'his', 'i', 'if', 'in', 'is', 'it', 'its', 'my', 'no', 'not', 'of', 'on', 'or', 'our', 'own', 'she', 'that', 'the', 'their', 'them', 'then', 'these', 'those', 'this', 'to', 'us', 'will', 'was', 'we', 'were', 'what', 'when', 'where', 'while', 'who', 'why', 'would', 'you', 'your']
#    return ['and', 'is', 'of', 'the', 'to']
#    return ['is', 'of', 'the', 'to', '.']
    return ['and', 'is', 'of', 'the', 'to', '.']

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

def get_and_remove_ocw_in_corpus(corpus, _OCW):
    new_corpus = []
    l_ocw_array = []
    for s in corpus:
        (ocw_array, l_sent_ccw) = extrac_open_class_words(l_sent=s, _OCW=_OCW, l_closed_class=get_closed_class_words())
        new_corpus.append(l_sent_ccw)
        l_ocw_array.append(ocw_array)
    if _OCW=='_':
        construction_words = get_closed_class_words()
    else:
        construction_words = get_closed_class_words()+[_OCW]
    return (new_corpus, l_ocw_array, construction_words)

def is_nr_ocw_in_construction_ok(construction, ocw_array, _OCW):
    """
    Checks if the number of OCW in ocw_array corresponds to the number of _OCW in the construction.
    If the number of OCW in construction is OK, then it return True.
    """
    if construction.count(_OCW) != len(ocw_array):
        #raise Warning, "The number of OCW in ocw_array do not corresponds to the number of _OCW in the construction"
        print "!!!WARNING!!! The number of OCW in ocw_array do not corresponds to the number of _OCW in the construction !!!"
        print "nr OCW in construciton",construction.count(_OCW)
        print "nr OCW in ocw_array", len(ocw_array)
        return False
    else:
        return True

def attribute_ocw_to_constructions(l_constructions, l_ocw_array, _OCW):
    print "---*** attribute_ocw_to_constructions ..."
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
        print "construction:",l_constructions[idx_c]
        print "OCW array:", l_ocw_array[idx_c]
        ocw_arr.reverse()
        for word in l_constructions[idx_c]:
            if word==_OCW:
                w = ocw_arr.pop()
                sent.append(w)
            else:
                sent.append(word)
        l_sent.append(sent)
    return l_sent

def get_canonical_order_for_single_meaning(meaning):
    """
    For the canonical order (SVO: Subject Verb Object in English and French) we just need to swap the Predicate and the Subject/Focus
    """
    print "  get_canonical_order_for_single_meaning:", [meaning[1], meaning[0]]+meaning[2:]
    return [meaning[1], meaning[0]]+meaning[2:]

def get_non_canonical_order1_for_single_meaning(meaning):
    """
    For the non canonical order 1 (VOS: Verb Object Subject/Focus in English) 
    ex: left violin trumpet -> to the left of the trumpet is the violin [left trumpet violin]
    ex: between violin trumpet drums -> between the trumpet and the drums is the violin [between trumpet drums violin]
    """
    print "  get_non_canonical_order1_for_single_meaning", [meaning[0]]+meaning[2:]+[meaning[1]]
    return [meaning[0]]+meaning[2:]+[meaning[1]]

def get_canonical_order_for_double_meaning(meanings):
    ## if the two focus of the two meanings match
    if meanings[0][1]==meanings[1][1]:
        ocw_array = [meanings[0][1]]
        # add meaning in canonical order without the subject/focus (1st element)
#        ocw_array.extend(get_canonical_order(meanings[0])[1:])
        ocw_array.extend(get_canonical_order_for_single_meaning(meanings[0])[1:])
#        ocw_array.extend(get_canonical_order(meanings[1])[1:])
        ocw_array.extend(get_canonical_order_for_single_meaning(meanings[1])[1:])
    else:
        raise Exception, "Cannot manage to generate a canonical order when focus elements are different. \n meanings:"+str(meanings)
    print "  get_canonical_order_for_double_meaning", ocw_array
    return ocw_array

def get_non_canonical_order1_for_double_meaning(meanings):
    """
    For the non canonical order 1 (VOS: Verb Object Subject/Focus in English) 
    ex: left violin trumpet,right violin guitar
            -> to the left of the trumpet and to the right of the guitar is the violin [left trumpet right guitar violin]
    ex: between violin trumpet drums, right violin guitar
            -> between the trumpet and the drums and to the right of the guitar is the violin [between trumpet drums right guitar violin]
    """
    ## if the two focus of the two meanings match
    if meanings[0][1]==meanings[1][1]:
        # add meaning in canonical order without the subject/focus (last element)
        ocw_array = (get_non_canonical_order1_for_single_meaning(meanings[0])[:-1])
        ocw_array.extend(get_non_canonical_order1_for_single_meaning(meanings[1])[:-1])
        # add the subject/focus at the end of the list
        ocw_array.append(meanings[0][1])
    else:
        raise Exception, "Cannot manage to generate a non canonical order 1 when focus elements are different. \n meanings:"+str(meanings)
    print "get_non_canonical_order1_for_double_meaning", ocw_array
    return ocw_array

def get_canonical_order(meanings):
    print " get_canonical_order ..."
    # there may be several meanings for each line of data
    if len(meanings)==1:
        ocw_array = get_canonical_order_for_single_meaning(meanings[0])
    elif len(meanings)==2:
        ocw_array = get_canonical_order_for_double_meaning(meanings=meanings)
    else:
        raise Exception, "Number of meaning not appropriate. Cannot manage 0 et more than 2."
    print " ... get_canonical_order, res=",ocw_array
    return ocw_array

def get_non_canonical_order1(meanings):
    print " get_non_canonical_order1 ..."
    # there may be several meanings for each line of data
    if len(meanings)==1:
        ocw_array = get_non_canonical_order1_for_single_meaning(meanings[0])
    elif len(meanings)==2:
        ocw_array = get_non_canonical_order1_for_double_meaning(meanings=meanings)
    else:
        raise Exception, "Number of meaning not appropriate. Cannot manage 0 et more than 2."
    print " ... get_non_canonical_order1, res=",ocw_array
    return ocw_array

def generate_l_ocw_array_in_canonical_order(l_meaning):
    print ""
    print "-* generate_l_ocw_array_in_canonical_order ... *-"
    l_ocw_array = []
    for meanings in l_meaning:
        ocw_array = get_canonical_order(meanings=meanings)
        l_ocw_array.append(ocw_array)
    print "-* ... generate_l_ocw_array_in_canonical_order *-"
    print ""
    return l_ocw_array

def generate_l_ocw_array_in_specified_order(l_meaning, l_sent_form):
    print ""
    print "-* generate_l_ocw_array_in_specified_order ..."
    l_ocw_array = []
    for idx_meanings in range(len(l_meaning)):
        if l_sent_form[idx_meanings] is not None:
            if l_sent_form[idx_meanings]==0:
                ocw_array = get_canonical_order(meanings=l_meaning[idx_meanings])
            elif l_sent_form[idx_meanings]==1:
                ocw_array = get_non_canonical_order1(meanings=l_meaning[idx_meanings])
            else:
                raise Exception, "Sentence form (canonical / non-canonical) not understood:"+str(l_sent_form[idx_meanings])
        else:
            ocw_array = get_canonical_order(meanings=l_meaning[idx_meanings])
        l_ocw_array.append(ocw_array)
    print "... generate_l_ocw_array_in_specified_order"
    print ""
    return l_ocw_array

def get_meaning_coding(max_nr_ocw=8, max_nr_actionrelation=2, elt_pred=['P','F','O','R']):
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

def generate_meaning_stim(l_data, l_ocw_array, full_time, l_offset=None, l_m_elt=None,
                          elt_pred=['P','F','O','R'], ocw_predicate_matches_with_one_meaning=True,
                          verbose=False):
    """ 
    
    Inputs:
        - 
        - 
        - 
        -ocw_predicate_matches_with_one_meaning: each predicate should match with only one meaning
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
        print "   m_code", m_code
        idx = l_m_elt.index(m_code)
        indices_m_code.append(idx)
        print "   idx", idx
        stim_seq[:full_time,idx] = np.ones((full_time,1)).T
        meaning_code.append(m_code)
    
    print "** generate_meaning_stim ..."
    
    
#    if subset is None:
#        subset = range(len(l_data))
    if l_m_elt is None:
        l_m_elt = get_meaning_coding()

#    stim = len(l_data)*[np.zeros((len(l_m_elt), full_time))]
#    stim = len(l_data)*[np.zeros((full_time,len(l_m_elt)))]
    stim = []
    l_meaning_code = []
    l_indices_m_code = []
    for seq_id in range(len(l_data)):
        # For each sentence
        print " *** seq_id "+str(seq_id)+" ... ***"
        meanings = l_data[seq_id]
        ocw_array = l_ocw_array[seq_id]
        stim_seq = np.zeros((full_time,len(l_m_elt)))
        meaning_code = []
        indices_m_code = []
        print " meanings", meanings
        print " ocw_array", ocw_array
        if is_there_several_time_the_same_elt_in_list(l=ocw_array):
            print "  ! There is several times the same OCW(s) in the current ocw_array."
            print "  ! This(These) Open Class Words is(are):"+str(is_there_several_time_the_same_elt_in_list(l=ocw_array))
        ## TODO: it may be more effecient reversing the order of the "for" loops (interverting idx_m with idx_ocw)
        ## New-way: for loop on OCW, then for loop on meaning
        #For each Open Class Word in ocw_array
        dic_predicate_already_attributed = {} # dictionary of predicates already attributed
        for idx_ocw in range(len(ocw_array)):
            # For each open class word in ocw_array
            print "  idx_ocw", idx_ocw
            print "  l_ocw_array[idx_ocw]", ocw_array[idx_ocw]
            for idx_m in range(len(meanings)):
                # For each meaning
                print "   idx_m", idx_m
                print "   meanings[idx_m]", meanings[idx_m]
                if ocw_array[idx_ocw] in meanings[idx_m]:
                    print "   meanings[idx_m].index(ocw_array[idx_ocw])", meanings[idx_m].index(ocw_array[idx_ocw])
                    i_action = meanings[idx_m].index(ocw_array[idx_ocw])
                    elt_p = elt_pred[i_action]
                    print "   elt_p", elt_p
                    # if element is a predicate and if each predicate should match with only one meaning
                    if elt_p=='P' and ocw_predicate_matches_with_one_meaning:
                        # if the predicate has already been coded/attributed
                        if (idx_ocw in dic_predicate_already_attributed.keys()) or (idx_m in dic_predicate_already_attributed.values()):
                            print "   !predicate "+str(ocw_array[idx_ocw])+" has already been attributed to meaning "+str(meanings[idx_m])
                            print "     dic_predicate_already_attributed:"+str(dic_predicate_already_attributed)
                        else:
                            # add this predicate with meaning to which at has been attributed
#                            dic_predicate_already_attributed.append(idx_ocw)
                            dic_predicate_already_attributed[idx_ocw] = idx_m
                            make_stim()
                    else:
                        make_stim()
                else:
                    print "    -X- "+str(ocw_array[idx_ocw])+" not in "+str(meanings[idx_m])
            print "  indices_m_code:",indices_m_code
        ## Old-way: for loop on meaning, then for loop on OCW
#        for idx_m in range(len(meanings)):
#            # For each meaning
#            print "  idx_m", idx_m
#            print "  meanings[idx_m]", meanings[idx_m]
#            for idx_ocw in range(len(ocw_array)):
#                # For each open class word in ocw_array
#                if ocw_array[idx_ocw] in meanings[idx_m]:
#                    print "   idx_ocw", idx_ocw
#                    print "   l_ocw_array[idx_ocw]", ocw_array[idx_ocw]
#                    print "   meanings[idx_m].index(ocw_array[idx_ocw])", meanings[idx_m].index(ocw_array[idx_ocw])
#                    i_action = meanings[idx_m].index(ocw_array[idx_ocw])
#                    elt_p = elt_pred[i_action]
#                    print "   elt_p", elt_p
#                    m_code = '_'+str(idx_ocw+1)+'-'+str(elt_p)+str(idx_m+1)
#                    print "   m_code", m_code
#                    idx = l_m_elt.index(m_code)
#                    indices_m_code.append(idx)
#                    print "   idx", idx
##                    stim[seq_id][idx,:] = np.ones((1, full_time))
##                    print "   np.ones((full_time,1)).shape", np.ones((full_time,1)).shape
##                    print "   stim[seq_id][:full_time,idx].shape", stim[seq_id][:full_time,idx].shape
#                    # we need to do the transpose (.T) of np.ones because stim[seq_id][:full_time,idx] is a vector of shape (full_time,)
#                    stim_seq[:full_time,idx] = np.ones((full_time,1)).T
##                    stim[seq_id][:full_time,idx] = np.ones((full_time,1)).T
#                    meaning_code.append(m_code)
#                else:
#                    print "    -X- "+str(ocw_array[idx_ocw])+" not in "+str(meanings[idx_m])
#            print "  indices_m_code:",indices_m_code
        stim.append(stim_seq)
        l_meaning_code.append(meaning_code)
        l_indices_m_code.append(indices_m_code)
        print " *** ... seq_id "+str(seq_id)+" ***"
    print "l_meaning_code", l_meaning_code
    print "l_indices_m_code", l_indices_m_code
    
    print "** ... generate_meaning_stim"
    return (stim, l_meaning_code)

def generate_1dim_sentence_like_stim(l_input, sentence, act_time, suppl_pause_at_the_end, full_time, pause=True, initial_pause=True, offset=None):
    """
    TODO: finish to code generate_1dim_sentence_like_stim()
    """
#    """
#    Returns a unidimensional stimulus which is like the longest sentence of corpus (this rely on full_time parameter)
#        The stim is equal to one when at least one word is active, and equals to zero the rest of the time.
#    
#    Inputs: 
#        - l_input: list of all possible words given in input. The length of this list gives the input dimension.
#        - full_time: whole number of time step of a stimulus (precedently it didn't include the initial_pause, now it include it)
#        - offset: represents the difference between the maximum number of words in the data and the number of word of a given sentence.
#            when taking into account the offset, offset has to be multiplied by 'act_time' (1 or 2 times depending if there 'pause' is false or true)
#        
#    If an offset is set, the '_' are not taken into account (there are not escape like spaces, but are taken like real words). Just a question of saving execution time.
#    
#    Variables:
#        - sentence[i]: i+1-th word of the current sentence
#        - l_input.index(sentence[i]): represents the index of the word 'sentence[i]' in the input stimulus
#    """
#    # Initializations
##TODO: suppress this addition of 2 act_time in case supplementary pause is None
#    if suppl_pause_at_the_end is None:
#        suppl_pause_at_the_end = 2*act_time # supplementary pause at the end of the sentence
#    if initial_pause is True:
#        j = 1
#    else:
#        j = 0
##    len_stim_in_words = len(sentence)
#    if pause==True:
#        #j = 1 #previously when initial_pause did not exist
#        time_pause = 1 # duration of a pause between the presentation of 2 words, in nb of 'act_time'. Set this value to 0 if you don't want to have pauses.
##        if full_time is None:
##            print "_stim_gen: Evaluating the full time of the stimulus."
##            mult = len_stim_in_words
##            #full_time = 2*act_time*mult
##            full_time = 2*act_time*mult + suppl_pause_at_the_end
#    else:
#        #j = 0 #previously when initial_pause did not exist
#        time_pause = 0
##        if full_time is None:
##            print "_stim_gen: Evaluating the full time of the stimulus."
##            mult = len_stim_in_words
##            #full_time = act_time*mult
##            full_time = act_time*mult + suppl_pause_at_the_end
#    stim = mdp.numx.zeros((len(l_input), full_time)) # stimulus (returned value)
#    # Generating the stimulus protocol while processing the sentence
##    for i in range(len_stim_in_words):
#    if offset is None:
#        for i in range(len(sentence)):
#            word = sentence[i]
#            if word == '_':
#                pass # an underscore is for an absence of stimulus, so we do nothing
#            else:
#    #            print "word="+word
#                idx = l_input.index(word)
##                stim[idx][act_time*j:act_time*(j+1)] = mdp.numx.ones((1,act_time)) 
#                stim[idx, act_time*j:act_time*(j+1)] = mdp.numx.ones((1,act_time)) 
#            j = j + 1 + time_pause # each stimulus is separated by a pause
#    else:
#        j = j + offset*(1*(pause==False)+2*(pause==True))
##        print "offset", offset
##        print "act_time",act_time
##        print "sentence:", sentence
##        print "j", j
##        print "len(l_input)", len(l_input)
##        print "l_input", l_input
##        print "full_time", full_time
#        for i in range(len(sentence)):
##            print "i",i
##            print "sentence[i]", sentence[i]
##            print "act_time*j:act_time*(j+1)", str(act_time*j)+':'+str(act_time*(j+1))
#            if sentence[i] == '_':
#                pass # an underscore is for an absence of stimulus, so we do nothing
#            else:
#                stim[l_input.index(sentence[i]), act_time*j:act_time*(j+1)] = mdp.numx.ones((1,act_time)) 
#            j = j + 1 + time_pause # each stimulus is separated by a pause
#    return stim.T

### Teaching and testing methods ###
####################################

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

def convert_one_output_activity_in_construction(out_act, construction_words, min_nr_of_val_upper_thres=1):
    """
    Inputs:
        - min_nr_of_val_upper_thres : number of values upper the threshold needed to take into account the word
            The default value is 1: this indicates that this parameters is useless,
                because 1 occurrence of an index is enough to add the corresponding word in the sentence.
            For instance if min_nr_of_val_upper_thres equals 1, it will not take into account singular
                pics into account.
    
    """
    # Each vector should be like this : stim_seq = np.zeros((full_time,len(construction_words)))
    
    signal_indices_max = convert_output_activity_in_signal_idx_max(out_act,  thres=0.4, eps = 1e-12)
    print "signal_indices_max:", signal_indices_max
    #Remove all minus indices (indices indicating that maximum value is not available
    while True:
        try:
            signal_indices_max.remove(-1)
        except:
            break
    while True:
        try:
            signal_indices_max.remove(-2)
        except:
            break
    print "signal_indices_max:", signal_indices_max
    # Compress all indices (make one value of consecutives values)
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
                print "keep in memory this index: ", signal_indices_max[i]
                print " - nr_occurrence_same_index: ", nr_occurrence_same_index
            # if we have to wait for more occurrences of this index to take it into account
            if (min_nr_of_val_upper_thres-1-nr_occurrence_same_index) > 0:
                # keep the index in memory
                print " - still have to wait"
                keep_in_merory = signal_indices_max[i]
            else:
                # add the word corresponding to this index in the final sentence
                print "new idx detected:", signal_indices_max[i]
                word = construction_words[signal_indices_max[i]]
                print "corresponding word:", word
                sent.append(word)
                previous = signal_indices_max[i]
                # reinitialize temporary variables
                nr_occurrence_same_index = 0
                keep_in_merory = -1
    
    if sent==[]:
        raise Exception, "No words has been generated by the network. Output activity may be too low."
    
    return sent
    


def convert_l_output_activity_in_construction(l_out_act, construction_words, min_nr_of_val_upper_thres=1):
    l_sent = [] # list of list of words i.e. list of sentences
    for out_act in l_out_act:
        sent = convert_one_output_activity_in_construction(out_act, construction_words, min_nr_of_val_upper_thres=min_nr_of_val_upper_thres)
        l_sent.append(sent)
    return l_sent


### Main Methods ###
##########################

def main(path_file_in, path_file_out, plot=False, keep_internal_states=False, verbose=False):
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
                file = open(file_path, 'wb')
        if file is None:
            raise Exception, "No file given in input."
        
        for item in l:
            file.write("%s\n" % item)
            
        if file_path is not None:
            file.close()
    
#    import Common_Tools.io_language_coding as CtIolangcod
    import sys
    sys.path.append("../Common_Tools")
    print sys.path
    import io_language_coding as CtIolangcod
    
    sys.path.append("../iCub_language")
    
    sentence_to_meaning = False
    
    # Definning parameters of stimulus (in a dictionary)
    d = {}
    d['act_time'] = 5#10#2
    d['pause'] = True#False
    d['suppl_pause_at_the_end'] = 1*d['act_time']
    d['initial_pause'] = True#False#False
    d['offset'] = False#True
    
    # Parameters for reservoir
    N = 500#1000 #100
    sr = 2#3#3#2#1
    iss = 0.01#1
    leak = 0.75#0.5#0.05
    
    ## output dic
    #    d['start_teacher'] = 1#'end'
    
    
    ## Random parameters
    seed = 5#2#4#2
    # seed 2 works with 2 sentences : both with 1 relation, 1 Canonical, 1 Non-canonical
    if seed is not None:
        mdp.numx.random.seed(seed)
        np.random.seed(seed)
#    if verbose:
#        print "Spectra radius of generated matrix before applying another spectral radius: "+str(Oger.utils.get_spectral_radius(w))
#    if spectral_radius is not None:
#        w *= d['spectral_radius'] / Oger.utils.get_spectral_radius(w)
#        if verbose:
#            print "Spectra radius matrix after applying another spectral radius: "+str(Oger.utils.get_spectral_radius(w))
#    if randomize_seed_afterwards:
#        """ redifine randomly the seed in order to not fix the seed also for other methods that are using numpy.random methods.
#        """
#        import time
#        mdp.numx.random.seed(int(time.time()*10**6))
    
#    [train_data_txt, test_data_txt] = extract_data_io(path_file=path_file_in, sentence_to_meaning=sentence_to_meaning)
    [train_data_txt, test_data_txt, sent_form_info_test] = extract_data_io(path_file=path_file_in, sentence_to_meaning=sentence_to_meaning)
    print "**************************"
    print "train data_txt", train_data_txt
    print "test data_txt", test_data_txt
    print "sent_form_info_test", sent_form_info_test
    train_corpus, train_meaning  = txt2corpus_and_meaning(train_txt=train_data_txt)
    if sentence_to_meaning:
        test_corpus = test_data_txt
    else:
        test_meaning = test_data_txt
    # making the list of constructions (refering to "construction grammar"), a construction is a sentence without its open class words (Nouns and Verbs)
    (l_construction_train, l_ocw_array_train, construction_words) = get_and_remove_ocw_in_corpus(corpus=train_corpus, _OCW='X')
    print "**************************"
    print "l_construction_train", l_construction_train
    print "l_ocw_array_train", l_ocw_array_train
    if sentence_to_meaning:
        (l_construction_test, l_ocw_array_test, construction_words_test) = get_and_remove_ocw_in_corpus(corpus=test_corpus, _OCW='X')
        print "l_construction_test", l_construction_test
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
    print "l_ocw_array_test", l_ocw_array_test
    
    ## Generating all the sentence stimulus (in order to have the same length for each sentence)
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
    
    print "stim_sent_train[0].shape", stim_sent_train[0].shape
    print "stim_sent_train[0].shape[0]", stim_sent_train[0].shape[0]
    
    l_m_elt = get_meaning_coding()
    print ""
    print "*** Generating meaning for train set ... ***"
    (stim_mean_train, l_meaning_code_train) = generate_meaning_stim(l_data=train_meaning, l_ocw_array=l_ocw_array_train, full_time=stim_sent_train[0].shape[0], l_m_elt=l_m_elt, verbose=False)
    print "*** ... meaning generated for train set ***"
    print "l_m_elt", l_m_elt
    print "stim_mean_train[0].shape", stim_mean_train[0].shape
    print "l_meaning_code_train", l_meaning_code_train
    print ""
    if not sentence_to_meaning:
        print "*** Generating meaning for test set ... ***"
        (stim_mean_test, l_meaning_code_test) = generate_meaning_stim(l_data=test_meaning, l_ocw_array=l_ocw_array_test, full_time=stim_sent_train[0].shape[0], l_m_elt=l_m_elt, verbose=False)
        print "*** ... meaning generated for test set ***"
        print ""
    
    reservoir = Oger.nodes.LeakyReservoirNode(output_dim = N, spectral_radius = sr, input_scaling =iss, nonlin_func = np.tanh, leak_rate = leak)
    read_out = mdp.nodes.LinearRegressionNode(use_pinv=True, with_bias=True)
    flow = mdp.Flow([reservoir, read_out])
    if keep_internal_states:
        Oger.utils.make_inspectable(mdp.Flow)
    
    print "Train and test"
#    (states_out_test, internal_states_test, internal_outputs_test, neuron_states_test) = \
    ## test set = train set
    (states_out_train, internal_states_train, internal_outputs_train, neuron_states_train) = \
        _teach_and_test_flow(inputs_train_set=stim_mean_train, teacher_outputs_train_set=stim_sent_train, inputs_test_set=stim_mean_train, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)
    ## test set not train set
    (states_out_test, internal_states_test, internal_outputs_test, neuron_states_test) = \
        _test_flow(inputs_test_set=stim_mean_test, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)
#    (states_out_test, internal_states_test, internal_outputs_test, neuron_states_test) = \
#        _teach_and_test_flow(inputs_train_set=stim_mean_train, teacher_outputs_train_set=stim_sent_train, inputs_test_set=stim_mean_test, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)
    
    
    for i in range(len(stim_mean_train)):
        print "len(stim_mean_train)", len(stim_mean_train)
        print "len(l_meaning_code_train)", len(l_meaning_code_train)
        print l_meaning_code_train[i]
        print (stim_mean_train[0]==stim_mean_train[i])
        print (l_meaning_code_train[0]==l_meaning_code_train[i])
    
    # Ecriture de la phrase de réponse
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
    print ""
    print "**********************************************"
    print "*** Processing recovery of test sentences ..."
    l_recovered_construction_test = convert_l_output_activity_in_construction(l_out_act=states_out_test,
                                                                              construction_words=construction_words,
                                                                              min_nr_of_val_upper_thres=2)
    l_recovered_sentences_test = attribute_ocw_to_constructions(l_constructions=l_recovered_construction_test,
                                                                l_ocw_array=l_ocw_array_test, _OCW='X')
    print "*** l_recovered_sentences_test: ***"
    for s in l_recovered_sentences_test:
        print s
    print "**********************************************"
    
    
    ## Writting sentences to output file
    print " *** Writting to output file ... *** "
    l_final_sent_test = []
    for list_words in l_recovered_sentences_test:
        l_final_sent_test.append(" ".join(list_words))
    #ecrire une seule ligne simple dans un fichier la phrase attendue en mode test
    write_list_in_file(l=l_final_sent_test, file_path=path_file_out)
    print " *** ... Writting done ***"
    print "**********************************************"
    
    
    ## Plot inputs
    if plot:
        print " *** Plotting to output file ... *** "
        import oct2011.plotting as plotting
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/test_sent_train", array_=stim_sent_train, plot_slice=None, title="", subtitle="")
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/test_sent_test", array_=stim_sent_test, plot_slice=None, title="", subtitle="")
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/test_mean_train0", array_=stim_mean_train[0].T, plot_slice=None, title="", subtitle="")
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/test_mean_train1", array_=stim_mean_train[1].T, plot_slice=None, title="", subtitle="")
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/test_mean_train_T", array_=stim_mean_train[0].T, plot_slice=None, title="", subtitle="")
    
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_train", array_=states_out_train, titles_subset=l_construction_train, legend_=construction_words, plot_slice=None, title="", subtitle="")
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_train_recov", array_=states_out_train, titles_subset=l_recovered_sentences_train, legend_=construction_words, plot_slice=None, title="", subtitle="")
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_train_detail", array_=states_out_train[0].T, titles_subset=l_construction_train[0], plot_slice=None, title="", subtitle="")
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_test", array_=states_out_test, titles_subset=l_recovered_sentences_test, legend_=construction_words, plot_slice=None, title="", subtitle="")
    
    
        ## Plot internal states
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/intern_states_train", array_=internal_states_train, titles_subset=l_construction_train, plot_slice=None, title="", subtitle="")
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/intern_states_test", array_=internal_states_test, titles_subset=l_ocw_array_test, plot_slice=None, title="", subtitle="")
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/test_int_states", array_=out, plot_slice=None, title="", subtitle="")
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/test_int_states_T", array_=out.T, plot_slice=None, title="", subtitle="")
        print " *** ... Plotting to output file done *** "
        print "**********************************************"
        
        ## Plot outputs
    #    plotting.plot_array_in_file(root_file_name="../../RES_TEMP/output_for_train_set", array_=states_out_train, data_subset=range(len(states_out_train)), plot_slice=None, title="", subtitle="")

    

if __name__ == '__main__':
    mode = 'real for icub'#'real for icub'
    
    main(path_file_in="../../../SR_input_M.txt", path_file_out="../../../SR_output_S.txt", plot=False, keep_internal_states=False)
    print "*********END OF PROGRAM********"
