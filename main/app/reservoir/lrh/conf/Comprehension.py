# -*- coding: utf-8 -*-
"""
Created on 11 janv. 2012

@author: Xavier HINAUT  Anne-Laure MEALIER

"""

import mdp
import Oger
import numpy as np
#import reservoir as res
import io_language_coding as CtIolangcod
import pickle 
import shelve 

class Comprehension:
    def __init__(self, corpusFile, fileResult, smode, closed_class_words, l_elt_pred, nbNeuron, verbose):
        self.corpusFile = corpusFile
        self.fileResult = fileResult
        self.smode = smode
        self.closed_class_words = closed_class_words 
        self.l_elt_pred= l_elt_pred
        self.nbNeurons = nbNeuron
        self.verbose = verbose
        
        self.mainFunc()
                
    def meaning_stim_to_meaning_code(self, stim, ocw_array, l_m_elt):
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
        if self.verbose:
            print "stim : ", stim
            print "ocw_array : ", ocw_array
            print "l_m_elt : ", l_m_elt
            print "self.l_elt_pred : ", self.l_elt_pred
            print "self.imax_nr_actionrelation : ", self.imax_nr_actionrelation
        ocws = ocw_array[:]
        ocws.reverse()
        meanings = []
        PAOR = []
        for i_m in range(0,self.imax_nr_actionrelation):
            meaning = []
            for i_elt_pred in range(0,len(self.l_elt_pred)):
                meaning.append(None)
            meanings.append(meaning)

            
        if self.verbose:
            print "self.l_elt_pred : ", self.l_elt_pred  #  ['P', 'A', 'O', 'R', 'V', 'W']
            print "  initial meanings:", meanings
            print "l_m_elt : ", l_m_elt
        # for each signal in stim/output
        for i in range(len(l_m_elt)):
            if stim[i]==1:
                m_elt = l_m_elt[i]
                pos1 = m_elt.find('_')
                pos2 = m_elt.find('-')
                idx_ocw = int(m_elt[pos1+1:pos2]) # looking at '#' in '_#-P1'

                try:
                    word = ocw_array[idx_ocw-1]
                except:
                    word = '_X_'
                    
                e_p = str(m_elt[pos2+1]) # looking at 'X' in '_1-X1'
                idx_elt_pred = self.l_elt_pred.index(e_p)
                idx_meaning = int(m_elt[pos2+2:]) # looking at '#' in '_1-P#'

                if (meanings[idx_meaning-1][idx_elt_pred] != word):
                    meanings[idx_meaning-1][idx_elt_pred] = word
                    tmp=[]
                    tmp.append(word)
                    tmp.append(m_elt.split("-",1)[1])
                    PAOR.append(tmp);                
                    
        for i_m in range(0,self.imax_nr_actionrelation):
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
    
    def convert_one_output_activity_in_meaning(self, out_act, ocw_array, l_m_elt, thres=0.6):
        # threshold the output
        act_thres = (out_act>thres)
        if self.verbose:
            print " act_thres : ",  act_thres
        # get the code from the last time time activity
        meanings = self.meaning_stim_to_meaning_code(stim=act_thres[-1,:], ocw_array=ocw_array, l_m_elt=l_m_elt)
        # get meanings with code and l_ocw_array
    #    meanings = code_to_meanings(code, ocw_array)
        return meanings
    
    def convert_l_output_activity_in_meaning(self, l_out_act, l_ocw_array, l_m_elt):
        l_meanings = [] # list of meanings
        for i_out in range(len(l_out_act)):
            meanings = self.convert_one_output_activity_in_meaning(l_out_act[i_out], l_ocw_array[i_out], l_m_elt=l_m_elt)
            l_meanings.append(meanings)
        if self.verbose:
            print "l_meanings : ", l_meanings
        return l_meanings

    
    def extract_data_io(self, path_file):
        
        train_flag = False
        test_flag = False
        if (self.smode == "train"):
            train = []
            f = open(path_file, "r")
            for line in f:
                (line_tmp,x,x)=line.partition('#')
                line_tmp = line_tmp.strip()

                if line_tmp=='<train data>':                    
                    train_flag = True
                elif line_tmp=='</train data>':
                    train_flag = False
                elif line_tmp=='':
                    pass
                elif (train_flag==True and test_flag==False):
                    x = self.extract_line_train(l=line_tmp)
                    train.append(x)
            if self.verbose:
                print "train : ", train
            f.close()
            return train 
            
        else:
            test = []
            f = open(path_file, "r")
            for line in f:
                (line_tmp,x,x)=line.partition('#')
                line_tmp = line_tmp.strip()
                
                if line_tmp=='<test data>':
                    test_flag = True
                elif line_tmp=='</test data>':
                    test_flag = False
                elif line_tmp=='':
                    pass
                elif (train_flag==False and test_flag==True):
                    x = self.extract_line_test(l=line_tmp)
                    test.append(x)
            
            print "test : ", test
            f.close()
            return test
            
                    
    def txt2corpus_and_meaning(self, train_txt):
        train_corpus, train_meaning  = [], []
        ## For each (sentence,meanings) tuple
        for (s,m) in train_txt:
            train_corpus.append(s)
            train_meaning.append(m)
        return train_corpus, train_meaning
        
    def get_and_remove_ocw_in_corpus(self, corpus, _OCW, l_closed_class):
        new_corpus = []
        l_ocw_array = []
        for s in corpus:
            (ocw_array, l_sent_ccw) = self.extrac_open_class_words(l_sent=s, _OCW=_OCW, l_closed_class=l_closed_class)
            new_corpus.append(l_sent_ccw)
            l_ocw_array.append(ocw_array)
        if _OCW=='_':
            construction_words = l_closed_class
        else:
            construction_words = l_closed_class+[_OCW]
        return (new_corpus, l_ocw_array, construction_words)


    def get_meaning_coding(self):
        l=[]
        for spred in range (0, len(self.focus)):
            if self.verbose:
                print "spred : ", self.focus[spred]
            for paor in range (0,len(self.focus[spred])):
                if self.verbose:
                    print "paor : ", self.focus[spred][paor]
                i=0
                for scaractere in self.focus[spred][paor]:
                    if scaractere != '_':
                        l.append('_'+str(spred+1)+'-'+str(scaractere)+str(i+1))
                    if scaractere == '_':
                        l.append('_'+str(spred+1)+'-'+'X'+str(i+1))
                    i+=1
        if self.verbose:
            print "l :", l
        return l  
        

    
    def get_meaning_coding(self):
                

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

        if self.verbose:
            print "self.l_elt_pred : ", self.l_elt_pred
        l = []
        for i in range(1,self.imax_nr_ocw+1):
            for j in range(1,self.imax_nr_actionrelation+1):
                for elt_p in self.l_elt_pred:
                    l.append('_'+str(i)+'-'+str(elt_p)+str(j))
        return l
    
    def generate_meaning_stim(self, l_data, l_ocw_array, full_time, l_offset=None, l_m_elt=None,
                              elt_pred=['P','A','O','R','V'], ocw_predicate_matches_with_one_meaning=False,
                              initial_pause=True, pause=True, act_time=None):
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
            idx = l_m_elt.index(m_code)
            indices_m_code.append(idx)
            if l_offset is None:
                stim_seq[:full_time,idx] = np.ones((full_time,1)).T
            else:
                st = l_start_TS[seq_id]
                stim_seq[:st,idx] = np.zeros((st,1)).T
                stim_seq[st:full_time,idx] = np.ones((full_time-st,1)).T
            meaning_code.append(m_code)
        
#            meaning_code=[]
#            for spred in range (0, len(self.focus)):
#                if self.verbose:
#                    print "spred : ", self.focus[spred]
#                for paor in range (0,len(self.focus[spred])):
#                    if self.verbose:
#                        print "paor : ", self.focus[spred][paor]
#                    i=0
#                    for scaractere in self.focus[spred][paor]:
#                        if scaractere != '_':
#                            meaning_code.append('_'+str(spred+1)+'-'+str(scaractere)+str(i+1))
#                        if scaractere == '_':
#                            meaning_code.append('_'+str(spred+1)+'-'+'X'+str(i+1))
#                        i+=1
#            if self.verbose:
#                print "l :", meaning_code
        
        def get_starting_point_in_act_time(offset):
            """
            starting point in act_time (Activation Time)
            """
            return initial_pause*1 + offset*(1*(pause==False)+2*(pause==True))
        
        print "** generate_meaning_stim ..."
        
        
    #    if subset is None:
    #        subset = range(len(l_data))
        if l_m_elt is None:
            l_m_elt = self.get_meaning_coding()
    
        if l_offset is not None:
            if act_time is None:
                raise Exception, "Could not use offset information to generate data, act_time information is compulsory to use the offset."
            l_start_TS = [] # start in time step
            for i_os in range(len(l_offset)):
                start_AT = get_starting_point_in_act_time(l_offset[i_os])
                l_start_TS.append(start_AT*act_time)
        if self.verbose:    
            print "l_m_elt : ", l_m_elt   
        # Need to be clean
            
    #    stim = len(l_data)*[np.zeros((len(l_m_elt), full_time))]
    #    stim = len(l_data)*[np.zeros((full_time,len(l_m_elt)))]
        stim = []
        l_meaning_code = []
        l_indices_m_code = []
        for seq_id in range(len(l_data)):
            # For each sentence
            meanings = l_data[seq_id]
            ocw_array = l_ocw_array[seq_id]
            stim_seq = np.zeros((full_time,len(l_m_elt)))
            meaning_code = []
            indices_m_code = []
            if self.is_there_several_time_the_same_elt_in_list(l=ocw_array):
                if self.verbose:
                    print "  ! There is several times the same OCW(s) in the current ocw_array."
                    print "  ! This(These) Open Class Words is(are):"+str(self.is_there_several_time_the_same_elt_in_list(l=ocw_array))
            ## TODO: it may be more effecient reversing the order of the "for" loops (interverting idx_m with idx_ocw)
            ## New-way: for loop on OCW, then for loop on meaning
            #For each Open Class Word in ocw_array
            dic_predicate_already_attributed = {} # dictionary of predicates already attributed
            for idx_ocw in range(len(ocw_array)):
                # For each open class word in ocw_array
                for idx_m in range(len(meanings)):
                    # For each meaning
                    if ocw_array[idx_ocw] in meanings[idx_m]:
                        i_action = meanings[idx_m].index(ocw_array[idx_ocw])
                        elt_p = elt_pred[i_action]

                        # if element is a predicate and if each predicate should match with only one meaning
                        if elt_p=='P' and ocw_predicate_matches_with_one_meaning:
                            # if the predicate has already been coded/attributed
                            if (idx_ocw in dic_predicate_already_attributed.keys()) or (idx_m in dic_predicate_already_attributed.values()):
                                if self.verbose:
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


        # Anne            
        l_meaning_code = []
        for listM in l_data:
            l_meaning_temp = []

            if len(listM)%2 == 0:
                tabMean = listM[:(len(listM)/2)]
                tabFocus = listM[(len(listM)/2):]
                for i in range (len(tabFocus)):                   
                    for j in range (len(tabFocus[i])):
                        if tabFocus[i][j] != '_':
                            code = '_' +  str(j+1) + '-' + tabFocus[i][j] + str(i+1)
                            l_meaning_temp.append(code)
                l_meaning_code.append(l_meaning_temp)
            else:
                print "error  ", listM
        
        print "** ... generate_meaning_stim"
        

#        stimuli=[]
#        verbose = True
#        
#        for spred in range (0, len(self.focus)):
#            temp = []
#            for paor in range (0,len(self.focus[spred])):
#                temp2 = []        
#                for scaractere in self.focus[spred][paor]:
#                    if scaractere != '_':
#                        temp2.append(1.)
#                    else:
#                        temp2.append(0.)
#                temp.append(temp2)
#            stimuli.append(np.array(temp))
#    
        return (stim, l_meaning_code)

    def is_there_several_time_the_same_elt_in_list(self, l):
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

    def extrac_open_class_words(self, l_sent, _OCW, l_closed_class):
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
    
    def test_flow(self, inputs_test_set, _flow, _reservoir, keep_internal_states=False, type_reservoir="", reset_state_before_begin=None):
        """
        TODO: change the use of type_reservoir, used _reservoir instead
        """
        # _flow :  [LeakyReservoirNode, LinearRegressionNode]
        # _reservoir :  LeakyReservoirNode      
        
        states_out = len(inputs_test_set)*[None]
        internal_states = []
        internal_outputs = []
        neuron_states = []
        
        if keep_internal_states==True:
            if type_reservoir == "LeakyReservoirNode" or (type(_reservoir) is Oger.nodes.LeakyReservoirNode):
                internal_states = len(inputs_test_set)*[None]
            else:
                #raise exception
                print "No reservoir type given in _test_flow()"
        # Testing the flow for each input in the test set
                
        for idx_out in range(len(inputs_test_set)):                
            states_out[idx_out] = _flow(inputs_test_set[idx_out])                        
        return (states_out, internal_states, internal_outputs, neuron_states)
   
    def teach_flow(self, inputs, teacher_outputs, _flow, _reservoir=None, reset_state_before_begin=None):
        """
        TODO: see if the reset of the states (when reset_state_before_begin is true) if working
        """
        if reset_state_before_begin==True:
            _reservoir.initial_states = mdp.numx.zeros((1, _reservoir.output_dim))
            
        learning_data = [inputs, zip(inputs, teacher_outputs)]
        _flow.train(learning_data)
        return _flow   
   
    def teach_and_test_flow(self, inputs_train_set, teacher_outputs_train_set, inputs_test_set, _flow, _reservoir, keep_internal_states=False,
                            type_reservoir="", return_flow=False, reset_state_before_begin=None):
        """
        return _flow  
        A 'Flow' is a sequence of nodes that are trained and executed together to form a more complex algorithm.
        Input data is sent to the first node and is successively processed by the subsequent nodes along the sequence.
        """
        flow_trained = self.teach_flow(inputs=inputs_train_set, teacher_outputs=teacher_outputs_train_set, _flow=_flow, _reservoir=_reservoir, reset_state_before_begin=reset_state_before_begin)
        
        
        
        (states_out, internal_states, internal_outputs, neuron_states) = self.test_flow(inputs_test_set=inputs_test_set, _flow=flow_trained, _reservoir=_reservoir, keep_internal_states=keep_internal_states, type_reservoir=type_reservoir, reset_state_before_begin=reset_state_before_begin)

        if return_flow:
            return (states_out, internal_states, internal_outputs, neuron_states, flow_trained)
        else:
            return (states_out, internal_states, internal_outputs, neuron_states)  # internal_states, internal_outputs, neuron_states => []
    
    
    def write_list_in_file(self, l, file=None, file_path=None):
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
                    file.seek(0)
		    file.truncate()
            if file is None:
                raise Exception, "No file given in input."
            
            for item in l:
                file.write("%s\n" % item)
                
            if file_path is not None:
                file.close()

    def extract_line_train(self, l):
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
        m_res = self.extr_meaning(meaning)
        
        # processing sentence
        # ex: the guitar over the violin is
        s_res = self.extr_sent(sent=sentence)

        return (s_res, m_res)
    
    def extract_line_test(self, l):
        print " extracting 1 line of test"
        if ';' in l:
            print "current line:", l
            raise Exception, "Ambiguous line: there should be no ';' because it is a line in <test> ... </test>"
        return self.extr_sent(sent=l)

    def extr_sent(self, sent):
        sent = sent.strip()
        if len(sent)==0:
            raise Exception, " No words in sentence."
        return sent.split(' ')

    def extr_meaning(self, meaning):
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

    def serialize(self,data, namePickle):
        output = open(namePickle, 'w')
        pickle.dump(namePickle, output)    #p = pickle.Pickler(output) then p.dump(data)                                     
        output.close()
        
    def unserialize(self, namePickle):
        pic = open(namePickle, 'r')
        data = pickle.load(input)
        pic.close()
        return data
   
    def shelveData(self,data, namePickle):
        base = shelve.open(namePickle)
        base[namePickle]=data
        base.close()
    
    def unshelveData(self,data, namePickle):
        base = shelve.open('base')
        return base
    
    def display(self, states_out_train, states_out_test, l_meaning_code_train, train_meaning, l_final_mean_test, internal_states_test):
        print " *** Plotting to output file ... *** "
        import oct2011.plotting as plotting
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_train",
                                    array_=states_out_train, titles_subset=l_meaning_code_train,
                                    # legend_=l_m_elt, plot_slice=None, title="", subtitle="")
                                    legend_=None, plot_slice=None, title="", subtitle="")
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_train_sent",
                                    array_=states_out_train, titles_subset=train_meaning,
                                    # legend_=l_m_elt, plot_slice=None, title="", subtitle="")
                                    legend_=None, plot_slice=None, title="", subtitle="")
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/states_out_test",
                                    array_=states_out_test, titles_subset=l_final_mean_test,
                                    # legend_=l_m_elt, plot_slice=None, title="", subtitle="")
                                    legend_=None, plot_slice=None, title="", subtitle="")   
    
        plotting.plot_array_in_file(root_file_name="../../RES_TEMP/intern_states_test", array_=internal_states_test, titles_subset=None, plot_slice=None, title="", subtitle="")

        print " *** ... Plotting to output file done *** "
        print "**********************************************"

    
    def train(self, d, train_data, focus, sr=1, iss=0.25, leak=0.25/5.0, seed=5, plot=False, fast=False, keep_internal_states=False):            
        train_corpus, train_meaning  = self.txt2corpus_and_meaning(train_txt=train_data)
        if self.verbose:
            print "train_corpus ", train_corpus
            print "train_meaning ", train_meaning
        ## Random parameters
        if seed is not None:
            mdp.numx.random.seed(seed)
            np.random.seed(seed)
            
        # making the list of constructions (refering to "construction grammar"), a construction is a sentence without its open class words (Nouns and Verbs)
        (l_construction_train, l_ocw_array_train, construction_words) = self.get_and_remove_ocw_in_corpus(corpus=train_corpus, _OCW='X', l_closed_class=self.closed_class_words)          
                        
        #################################################
        ## Generating all the sentence stimulus (in order to have the same length for each sentence)
       
        l_full_train = l_construction_train        
        slice_train = slice(0,len(l_construction_train))
        if self.verbose:
            print "slice_train: ", slice_train               
            print "l_construction_train : ", l_construction_train
            
        (stim_full_data_train, l_full_offset_train) = CtIolangcod.generate_stim_input_nodic(l_data=l_full_train,
                                act_time=d['act_time'], subset=None, l_input=construction_words,
                                l_nr_word=None, mult=None, full_time=None,
                                with_offset=d['offset'], pause=d['pause'], initial_pause=d['initial_pause'],
                                suppl_pause_at_the_end=d['suppl_pause_at_the_end'])
                                
        stim_sent_train = stim_full_data_train[slice_train]
        
        #################################################
        ## Generating all the meaning stimulus 
        #################################################
    
    

    
        l_m_elt = self.get_meaning_coding()
    
        (stim_mean_train, l_meaning_code_train) = self.generate_meaning_stim(l_data=train_meaning,
               l_ocw_array=l_ocw_array_train, full_time=stim_sent_train[0].shape[0],
               l_m_elt=l_m_elt, l_offset=l_full_offset_train[slice_train],
               initial_pause=d['initial_pause'], pause=d['pause'], act_time=d['act_time'])
        
        
        ## Defining reservoir, readout and flow
        reservoir = Oger.nodes.LeakyReservoirNode(output_dim = self.nbNeurons, spectral_radius=sr, input_scaling=iss, nonlin_func=np.tanh, leak_rate=leak)
     
        read_out = mdp.nodes.LinearRegressionNode(use_pinv=True, with_bias=True)
        flow = mdp.Flow([reservoir, read_out]) 

        ## Trainning and testing
        (states_out_train, internal_states_train, internal_outputs_train, neuron_states_train) = \
            self.teach_and_test_flow(inputs_train_set=stim_sent_train, teacher_outputs_train_set=stim_mean_train, inputs_test_set=stim_sent_train, _flow=flow, _reservoir=reservoir, keep_internal_states=keep_internal_states)

        return l_construction_train, stim_full_data_train, flow, reservoir, keep_internal_states, l_m_elt, construction_words

    def test(self, d, test_corpus, shelf):
               
        
        (l_construction_test, l_ocw_array_test, construction_words_test) = self.get_and_remove_ocw_in_corpus(corpus=test_corpus, _OCW='X', l_closed_class=self.closed_class_words)        

        if shelf["construction_words"]!=construction_words_test:
            raise Exception, "The construction words are not the same for the train constructions and the test constructions. So the coding of sentences will be different and should provoque a future problem."
   
        l_full_const = l_construction_test

        (stim_full_data_test, l_full_offset_test) = CtIolangcod.generate_stim_input_nodic(l_data=l_full_const,
                                act_time=d['act_time'], subset=None, l_input=shelf["construction_words"],
                                l_nr_word=None, mult=None, full_time=None,
                                with_offset=d['offset'], pause=d['pause'], initial_pause=d['initial_pause'],
                                suppl_pause_at_the_end=d['suppl_pause_at_the_end'], verbose=False)
                                
        slice_test = slice(len(shelf["l_construction_train"]),len(shelf["l_construction_train"])+len(l_construction_test))
        
        stim_full_data = shelf["stim_full_data_train"] + stim_full_data_test
        stim_sent_test = stim_full_data[slice_test]
        
        (states_out_test, internal_states_test, internal_outputs_test, neuron_states_test) = \
            self.test_flow(inputs_test_set=stim_sent_test, _flow=shelf["flow"], _reservoir=shelf["reservoir"], keep_internal_states=shelf["keep_internal_states"])

        l_recovered_meaning_test = self.convert_l_output_activity_in_meaning(l_out_act=states_out_test, l_ocw_array=l_ocw_array_test, l_m_elt=shelf["l_m_elt"])
        return l_recovered_meaning_test
        

    
    def mainFunc(self, plot=False):
        import os
        #sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/..")
        #print "path   ", os.path.dirname(os.path.abspath(__file__))+"/.."

#        current_directory = os.path.dirname(os.path.abspath(__file__))
#        parent_directory = os.path.dirname(current_directory)
#        sys.path.append(parent_directory)
#        

        # Definning parameters of stimulus (in a dictionary)
        d = {}
        d['act_time'] = 5#2#1#5#10#2
        d['pause'] = True#False
        d['suppl_pause_at_the_end'] = 1*d['act_time']
        d['initial_pause'] = False#True#False#False
        d['offset'] = True#False#True
        
        #leak=0.25/float(d['act_time'])        
        #N = 500

        data = self.extract_data_io(path_file=self.corpusFile)
        focus = []
        meaning = []
        sentence = []
        
        for i in range (0, len(data)):
            sentence.append(data[i][0])
            meaningFocus = data[i][1]
            meaning.append(meaningFocus[0:(len(meaningFocus)/2)])
            focus.append(meaningFocus[(len(meaningFocus)/2):])
            if self.verbose:
                print "focus : ", focus
                print "meaning : ", meaning 



        if self.verbose:
            print "################"
            print "data : ", data
            print "  "
            print "sentence : ", sentence
            print "focus : ", focus
            print "meaning : ", meaning
            print "################"
            print "         "
        
        
        sdir = os.path.dirname(os.path.abspath(self.fileResult))
        print "sdir main func : ", sdir
        
        if self.smode =="test":
            shelf = shelve.open(sdir + '/shelf.db', flag='r')
            print "sdir + '/shelf.db'", sdir 
            flag = shelf.has_key("l_construction_train")
            if flag:
                test_corpus = data               
                
                self.imax_nr_actionrelation = shelf["imax_nr_actionrelation"]
                self.imax_nr_ocw = shelf["imax_nr_ocw"]
                self.focus = shelf["focus"]
                l_recovered_meaning_test = self.test(d,test_corpus,shelf)
                ## Writting output meaning       
            
                if self.verbose:
                    print "l_recovered_meaning_test", l_recovered_meaning_test
                l_final_mean_test = []
                for meanings in l_recovered_meaning_test:
                    current_meanings = ""
                    if self.verbose:
                        print "meanings", meanings, shelf["construction_words"]
                    for i_m in range(len(meanings)):
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
                self.write_list_in_file(l=l_final_mean_test, file_path=self.fileResult)   
                shelf.close()
                print " *** ... Writting done ***"
                print "**********************************************"
            else:
               raise Exception, "The train was not launched"
    
        else:
            sentence = []
            meaningFocus = []
            meaning = []
            self.focus = []
            for i in range (0, len(data)):
                sentence.append(data[i][0])
                meaningFocus = data[i][1]
                meaning.append(meaningFocus[0:(len(meaningFocus)/2)])
                self.focus.append(meaningFocus[(len(meaningFocus)/2):])    
                
            self.imax_nr_ocw = 0
            self.imax_nr_actionrelation = 0

            for i in range (len(self.focus)): 
                itemp = 0
                temp_nr_actionrelation = len(self.focus[i])   
                for j in range (len(self.focus[i])):
                    for k in range (len(self.focus[i][j])):
                        if self.focus[i][j][k] != '_':
                            itemp+=1
                    if itemp > self.imax_nr_ocw:
                        self.imax_nr_ocw = itemp
                if temp_nr_actionrelation > self.imax_nr_actionrelation:
                    self.imax_nr_actionrelation = temp_nr_actionrelation
                    
            shelf = shelve.open(sdir + '/shelf.db', writeback=True)
            (l_construction_train, stim_full_data_train, flow, reservoir, keep_internal_states, l_m_elt, construction_words) = self.train(d, data, focus)
            
            shelf["l_construction_train"]=l_construction_train
            shelf["stim_full_data_train"]=stim_full_data_train
            shelf["flow"]=flow
            shelf["reservoir"]=reservoir
            shelf["keep_internal_states"]=keep_internal_states
            shelf["l_m_elt"]=l_m_elt
            shelf["construction_words"]=construction_words
            shelf["imax_nr_actionrelation"]= self.imax_nr_actionrelation
            shelf["imax_nr_ocw"]= self.imax_nr_ocw
            shelf["focus"]= focus
            shelf.close()
            
            
if __name__ == '__main__':
    import sys, os
    # global variables 
    sdir = os.path.dirname(os.path.abspath(__file__))
    print "sdir : ", sdir    
    corpusFile = sys.argv[1]
    fileResult = sys.argv[2]
    sMode = sys.argv[3]
    closed_class_wordsAP = sys.argv[4].split(',') # ['after', 'and', 'before', 'to', 'the', 'slowly', 'quickly', 'was', 'with', 'that', 'for', 'a', 'now']
    l_elt_pred= sys.argv[5].split(',')     #['P','A','O','R','V']
    nbNeurons = int(sys.argv[6])  # 500

    Comprehension(corpusFile, fileResult, sMode, closed_class_wordsAP, l_elt_pred, nbNeurons, False)

    
    sdir = os.path.dirname(os.path.abspath(__file__))

#    corpusFile = sdir + "/Corpus/Corpus2.txt"
#    #corpusFile = "/mnt/data/Dropbox/Reservoir_Workspace/Corpus/Corpus.txt"
#    fileResult = sdir + "/Corpus/output.txt"
#    closed_class_wordsAP = ['after', 'and', 'before', 'to', 'the', 'slowly', 'quickly', 'was', 'with', 'that', 'for', 'a', 'now']
#    #closed_class_wordsAP = ['wa','wo','no','ni','ga','you','koto']  
#    l_elt_pred = ['P','A','O','R','V','W']
#    nbNeurons = 500
#        
#    
#    lMode = ["train","test"]
#    for sMode in lMode:    
#        Comprehension(corpusFile, fileResult, sMode, closed_class_wordsAP, l_elt_pred, nbNeurons, False)


    print "*********END OF PROGRAM********"



