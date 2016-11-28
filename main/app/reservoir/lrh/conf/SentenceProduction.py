# -*- coding: utf-8 -*-
"""
Created on 11 janv. 2012

@author: Xavier HINAUT, Colas Droin, Anne-Laure MEALIER 
xavier.hinaut #/at\# inserm.fr
"""

import mdp
import numpy as np
import reservoir
import shelve


class Production:
    def __init__(self, corpusFile, fileResult, smode, closed_class_words, l_elt_pred, nbNeuron, verbose):
        self.corpusFile = corpusFile
        self.fileResult = fileResult
        self.smode = smode
        self.closed_class_words = closed_class_words
        self.l_elt_pred= l_elt_pred
        self.iNbNeurons = nbNeuron
        self.verbose = verbose
        
        self.mainFunc()

    def extr_sent(self, sent):
        sent = sent.strip() #removing spaces before and after the sentence
        if len(sent)==0:
            raise Exception, "No words in sentence."
        return sent.split(' ')
    
    ### Anne changes
    def extr_meaning(self, meaning, verbose=False):
        m_res = []
        a_res = []
        meaning = meaning.strip()
        meaning=meaning.split("<o>")[:-1]
    
        assignement = meaning[1]
        assignement = assignement.strip()
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
         
        a_res.append(assignement)
    
        a_res.append(assignement)
        return m_res, a_res, assignement
    
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
        m_res, a_res, assignement = self.extr_meaning(meaning, verbose=True)   
        
        # processing sentence
        # ex: the guitar over the violin is
        s_res = self.extr_sent(sent=sentence)
        return (s_res, m_res, a_res), (s_res, m_res), assignement
        
    def extract_line_test(self, l):
        if ';' in l:
            #print "current line:", l
            raise Exception, "Ambiguous line: there should be no ';' because it is a line in <test> ... </test>"
    
        if "<o>" in l:
            l = l.strip()
            #print "  m:["+str(l)+"]"
            m_res, a_res, assignement = self.extr_meaning(meaning=l, verbose=True)
            return (m_res, a_res) #assignement is currently replacing canonical information
        else:
            raise Exception, "Something is wrong in your train file, eg. \n <train data> \n\
            , wanted I , get I giraffe <o> [_-_-_-_-_-_-_-_][A-P-_-_-_-_-_-_][A-_-P-O-_-_-_-_] <o>; I wanted to get the giraffe \n\
            </train data>"
            
    
    
    def extract_data_io(self, path_file, verbose=False):
        flag_train = False
        flag_test = False
        
        train = []
        test = []
        sent_form_info_train = []
        sent_form_info_test = []
        
        f = open(path_file, "r")
        for line in f:
                if verbose:
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
                    if flag_train:     
                        x, y, z = self.extract_line_train(l=line_tmp)
                        train.append(y)
                        sent_form_info_train.append(z)
                    elif flag_test:
                        y = self.extract_line_test(l=line_tmp)
    
                        if type(y) is tuple:
                            assignement = y[1][0]
                            meaning = y[0]
                        test.append(meaning)
                        sent_form_info_test.append(assignement)
        f.close()
        return [train, test, sent_form_info_train, sent_form_info_test]
    
    
    
    ### Sentence & Meanning Methods ###
    ###################################
    
    def txt2corpus_and_meaning(self, train_txt):
        train_corpus, train_meaning  = [], []
        ## For each (sentence,meanings) tuple
        for (s,m) in train_txt:
            train_corpus.append(s)
            train_meaning.append(m)
        return train_corpus, train_meaning
        
    
    def extrac_open_class_words(self, l_sent, _OCW, l_closed_class=['after']):
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
    

    def get_and_remove_ocw_in_corpus(self, corpus, _OCW):
        new_corpus = []
        for s in corpus:
            (ocw_array, l_sent_ccw) = self.extrac_open_class_words(l_sent=s, _OCW=_OCW, l_closed_class=self.closed_class_words)
            new_corpus.append(l_sent_ccw)
        if _OCW=='_':
            construction_words = self.closed_class_words
        else:
            construction_words = self.closed_class_words+[_OCW]
        return (new_corpus, construction_words)
    
    def is_nr_ocw_in_construction_ok(self, construction, ocw_array, _OCW):
        """
        Checks if the number of OCW in ocw_array corresponds to the number of _OCW in the construction.
        If the number of OCW in construction is OK, then it returns True.
        """
        if construction.count(_OCW) != len(ocw_array):
            return False
        else:
            return True
    
    def attribute_ocw_to_constructions(self, l_constructions, l_ocw_array, _OCW):
        #print "---*** attribute_ocw_to_constructions ..."
        l_sent = []
        for idx_c in range(len(l_constructions)):
            sent = []
            ocw_arr = list(l_ocw_array[idx_c])
            if not self.is_nr_ocw_in_construction_ok(construction=l_constructions[idx_c], ocw_array=ocw_arr, _OCW=_OCW):
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
    
    #Anne changes
    def structure_partition(self, structure):
        m_res = []
        if ']' in structure:
            posPAOR = [i for i, letter in enumerate(structure) if letter == ']']
            nbPAOR = len(posPAOR)
            a = {}
            k = 0
            while k < nbPAOR:        
                key = "m" + k.__str__()
                if k==0:
                    a[key] = structure[1:posPAOR[k]].strip(" [],").split('-')
                elif k==nbPAOR:
                    a[key] = structure[posPAOR[k-1]+2:len(structure)].strip(" [],").split('-')
                else:
                    a[key] = structure[posPAOR[k-1]+2:posPAOR[k]].strip(" [],").split('-')
    
                m_res.append(a[key])
                k += 1
        return m_res
        
    
    def generate_l_ocw_array(self, tab_structure, tab_meaning):
        l_ocw_array = []
        for structure, meaning in zip(tab_structure, tab_meaning):
            ocw_array=[]
            cor={'P':0, 'A':1, 'O':2, 'R':3, 'V':4}
            m_res = self.structure_partition(structure)
    
            m_res = zip(*m_res) # transpose the list
    
            for tupl in m_res:
                i = 0
                for role in tupl:
                    i += 1
                    if role!='_':
                        if cor[role] >= len(meaning[i-1]):
                            ocw_array.append(meaning[i-1][len(meaning[i-1])-1])
    
                        else:
                            ocw_array.append(meaning[i-1][cor[role]])
                        break    
            l_ocw_array.append(ocw_array)
        return l_ocw_array
    
    
    def get_meaning_coding(self, max_nr_ocw=15, max_nr_actionrelation=5, elt_pred=['P','A','O','R','V']):
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
    
    #COLAS : function completely recoded
    def generate_meaning_stim(self, l_structure, full_time, l_m_elt):
    
        l_meaning_code = []
        l_indices_m_code = []
        stim = []
        for structure in l_structure:
            stim_seq = np.zeros((full_time,len(l_m_elt)))
            indices_m_code=[]
            meaning_code=[]
            m_res = self.structure_partition(structure)
            m_res = zip(*m_res) # transpose the list
            pos=0
            for tupl in m_res:
                n=0
                for role in tupl:
                    if role!='_':  
                        m_code = '_'+str(pos+1)+'-'+role+str(n+1)
                        idx = l_m_elt.index(m_code)
                        indices_m_code.append(idx)
                        stim_seq[:full_time,idx] = np.ones((full_time,1)).T
                        meaning_code.append(m_code)
                    n += 1
                pos += 1
            stim.append(stim_seq)
            l_meaning_code.append(meaning_code)
            l_indices_m_code.append(indices_m_code)
        return (stim, l_meaning_code)
    
    
    ### Teaching and testing methods ###
    ####################################
    #COLAS : all the teaching and testing function have been deleted. The didn't serve anymore thanks to the new reservoir.
    
    def convert_output_activity_in_signal_idx_max(self, out_act,  thres, eps):
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
    
    
    def convert_one_output_activity_in_construction(self, out_act, construction_words, min_nr_of_val_upper_thres=1):
        """
        Inputs:
            - min_nr_of_val_upper_thres : number of values upper the threshold needed to take into account the word
                The default value is 1: this indicates that this parameters is useless,
                    because 1 occurrence of an index is enough to add the corresponding word in the sentence.
                For instance if min_nr_of_val_upper_thres equals 2, it will not take into account singular
                    pics into account.
        
        """
        # Each vector should be like this : stim_seq = np.zeros((full_time,len(construction_words)))
        
        signal_indices_max = self.convert_output_activity_in_signal_idx_max(out_act,  thres=0.4, eps = 1e-12)
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
        
    def convert_l_output_activity_in_construction(self, l_out_act, construction_words, min_nr_of_val_upper_thres=1):
        l_sent = [] # list of list of words i.e. list of sentences
        sentence_nb=1
        for out_act in l_out_act:
            #print "training sentence number :", sentence_nb
            sent = self.convert_one_output_activity_in_construction(out_act, construction_words, min_nr_of_val_upper_thres=min_nr_of_val_upper_thres)
            l_sent.append(sent)
            sentence_nb+=1
        return l_sent
    
    #COLAS: treshold function to prevent the divergence of the reservoir (used with feedback)
    def treshold_signal(self, vect, sup, inf):
        for i in range(len(vect)):
            if vect[i]>=sup:    vect[i]=sup
            elif vect[i]<=inf:  vect[i]=inf
        return vect

    def write_list_in_file(self, listSentences, file=None, file_path=None):
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
        
        for item in listSentences:
            file.write("%s\n" % item)
            
        if file_path is not None:
            file.close()   
    
    def shelveData(self,data, namePickle):
        base = shelve.open(namePickle)
        base[namePickle]=data
        base.close()
    
    def unshelveData(self,data, namePickle):
        base = shelve.open('base')
        return base    

    def train(self, sent_form_info_train, train_meaning, train_corpus, d, sr=3, iss=0.1, leak=0.1, ridge=10**-1):
        import io_language_coding as CtIolangcod                
        ## Random parameters
        import time
        millis = int(round(time.time() ))    
        seed = millis#2#4#2
    
        if seed is not None:
            mdp.numx.random.seed(seed)
            np.random.seed(seed)

        # making the list of constructions (refering to "construction grammar"), a construction is a sentence without its open class words (Nouns and Verbs)
        (l_construction_train, construction_words) = self.get_and_remove_ocw_in_corpus(corpus=train_corpus, _OCW='X')
        l_ocw_array_train=self.generate_l_ocw_array(sent_form_info_train, train_meaning)

        l_full_const = l_construction_train
        slice_train = slice(0,len(l_construction_train))
        (stim_full_data, l_full_offset) = CtIolangcod.generate_stim_input_nodic(l_data=l_full_const,
    #                            act_time=d['act_time'], subset=None, l_input=None,
                                act_time=d['act_time'], subset=None, l_input=construction_words,
                                l_nr_word=None, mult=None, full_time=None,
                                with_offset=d['offset'], pause=d['pause'], initial_pause=d['initial_pause'],
                                suppl_pause_at_the_end=d['suppl_pause_at_the_end'], verbose=False)
        stim_sent_train = stim_full_data[slice_train]
        l_m_elt = self.get_meaning_coding(max_nr_ocw=self.imax_nr_ocw, max_nr_actionrelation=self.imax_nr_actionrelation, elt_pred=self.l_elt_pred)

        (stim_mean_train, l_meaning_code_train) = self.generate_meaning_stim(l_structure=sent_form_info_train, full_time=stim_sent_train[0].shape[0], l_m_elt=l_m_elt)

        # Reservoir and Read-out definitions
        res = reservoir.Reservoir(self.iNbNeurons, sr, iss, leak)
    
        #classic working of the reservoir without feedback

        ## test set = train set
        states_out_train, internal_states_train = res.train (stim_mean_train, stim_sent_train)

        return l_ocw_array_train, states_out_train, construction_words, internal_states_train, res, stim_mean_train, stim_sent_train, l_m_elt
        
# Feedback mode      
# feedback working of the reservoir. !! Should be implemented directly in the reservoir class !!
#        
#
#        delay=1
#        nb_epoch_max=4generate_meaning_stim
#        dim_input = stim_mean_train[0].shape[1]
#        #dim_output =  len(stim_sent_train[0][0])
#        input_train=[]
#
#        for (x,y) in zip( np.copy(stim_mean_train), np.copy(stim_sent_train)):
#            for time_step_delay in range(delay):
#                y=np.concatenate( ([[0.]*len(y[0])] , y), axis=0)
#            input_train.append(np.array(  np.concatenate(   (x, y[:-delay]), axis=1 )  ))
#      
#        nb_train=0  
#        while nb_train < nb_epoch_max:
#            ## test set = train set
#            states_out_train, internal_states_train = res.train (input_train, stim_sent_train)
#            
#            tab_feedback=[]
#            for num_phrase in range(len(states_out_train)):
#                #signal tresholded
#                states_out_train[num_phrase]=np.array([self.treshold_signal(signal_t,1.5,-0.5) for signal_t in states_out_train[num_phrase]])
#                if nb_train==0: #feedback kept only for the first train
#                    #feedback assignation
#                    feedback=np.array(states_out_train[num_phrase])
#                    #signal delayed
#                    for time_step_delay in range(delay):l_m_elt
#                        feedback=np.concatenate( ([[0.]*len(feedback[0])] , feedback), axis=0)
#                
#                tab_feedback.append(feedback)
#                input_train[num_phrase]=input_train[num_phrase].T
#                input_train[num_phrase][dim_input:] = feedback[:-delay].T
#                input_train[num_phrase]=input_train[num_phrase].T
#
#            nb_train+=1
#
#        ## test set not train set
#        for t in range(0,stim_mean_test[0].shape[0],1):
#            input_test=[]l_m_elt
#            if t==0: #A REMODIFIER
#                for n_phrase in range(len(stim_mean_test)):
#                    input_test.append(np.concatenate(  (stim_mean_test[n_phrase][t:t+1,:] , [[0.]*len(stim_sent_train[0][0])] ) , axis=1     ) )
#
#                states_out_test, internal_states_test = res.test(input_test)
#                import copy
#                states_out_test_def=copy.deepcopy(states_out_test)
#
#            else:
#                for n_phrase in range(len(stim_mean_test)):
#                    #feedback assignation
#                    feedback=np.array(states_out_test[n_phrase])
#                    input_test.append(np.concatenate(  (stim_mean_test[n_phrase][t:t+1,:] , feedback ) , axis=1     ) )
#
#                states_out_test, internal_states_test = res.test(input_test)
#            
#                for n_phrase in range(len(stim_mean_test)):
#                    states_out_test_def[ n_phrase ]=np.concatenate( (states_out_test_def[n_phrase] , states_out_test[n_phrase]), axis=0  )
#
#        states_out_test=states_out_test_def
            
    def test(self, test_corpus, sent_form_info_test, shelf, plot=False, feedback=False ):
        
        l_ocw_array_test = self.generate_l_ocw_array(sent_form_info_test, test_corpus)
        (stim_mean_test, l_meaning_code_test) = self.generate_meaning_stim(l_structure=sent_form_info_test, full_time=shelf["stim_sent_train"][0].shape[0], l_m_elt=shelf["l_m_elt"])

        
        #classic working of the reservoir
        if feedback==False:
            ## test set not train set
            states_out_test, internal_states_test = shelf["res"].test(stim_mean_test)
        #feedback working of the reservoir. !! Should be implemented directly in the reservoir class !!
        else:
            delay=1
            nb_epoch_max=4
            dim_input = shelf["stim_sent_train"][0].shape[1]
            #dim_output =  len(stim_sent_train[0][0])
            input_train=[]
    
            for (x,y) in zip( np.copy(shelf["stim_sent_train"]), np.copy(shelf["stim_sent_train"])):
                for time_step_delay in range(delay):
                    y=np.concatenate( ([[0.]*len(y[0])] , y), axis=0)
                input_train.append(np.array(  np.concatenate(   (x, y[:-delay]), axis=1 )  ))
          
            nb_train=0  
            while nb_train < nb_epoch_max:
                ## test set = train set
                states_out_train, internal_states_train = shelf["res"].train (input_train, shelf["stim_sent_train"])
                
                tab_feedback=[]
                for num_phrase in range(len(states_out_train)):
                    #signal tresholded
                    states_out_train[num_phrase]=np.array([self.treshold_signal(signal_t,1.5,-0.5) for signal_t in states_out_train[num_phrase]])
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
                        input_test.append(np.concatenate(  (stim_mean_test[n_phrase][t:t+1,:] , [[0.]*len(shelf["stim_sent_train"][0][0])] ) , axis=1     ) )
    
                    states_out_test, internal_states_test = shelf["res"].test(input_test)
                    import copy
                    states_out_test_def=copy.deepcopy(states_out_test)
    
                else:
                    for n_phrase in range(len(stim_mean_test)):
                        #feedback assignation
                        feedback=np.array(states_out_test[n_phrase])
                        input_test.append(np.concatenate(  (stim_mean_test[n_phrase][t:t+1,:] , feedback ) , axis=1     ) )
    
                    states_out_test, internal_states_test = shelf["res"].test(input_test)
                
                    for n_phrase in range(len(stim_mean_test)):
                        states_out_test_def[ n_phrase ]=np.concatenate( (states_out_test_def[n_phrase] , states_out_test[n_phrase]), axis=0  )
    
            states_out_test=states_out_test_def

#            l_recovered_construction_train = self.convert_l_output_activity_in_construction(l_out_act=states_out_train,
#                                                                                       construction_words=construction_words,
#                                                                                       min_nr_of_val_upper_thres=1)
#            l_recovered_sentences_train = self.attribute_ocw_to_constructions(l_constructions=l_recovered_construction_train,
#                                                                         l_ocw_array=l_ocw_array_train, _OCW='X')    
        l_recovered_construction_test = self.convert_l_output_activity_in_construction(l_out_act=states_out_test,
                                                                                  construction_words=shelf["construction_words"],
                                                                                  min_nr_of_val_upper_thres=2)
        l_recovered_sentences_test = self.attribute_ocw_to_constructions(l_constructions=l_recovered_construction_test,
                                                                    l_ocw_array=l_ocw_array_test, _OCW='X')
                                                                                    ## Plot inputs
        if plot:
            import plotting as plotting
        
            plotting.plot_array_in_file(root_file_name="../Results/states_out_train", array_=shelf["states_out_train"], titles_subset=shelf["l_construction_train"], legend_=shelf["construction_words"], plot_slice=None, title="", subtitle="")
    
            plotting.plot_array_in_file(root_file_name="../Results/states_out_test", array_=states_out_test, titles_subset=l_recovered_sentences_test, legend_=shelf["construction_words"], plot_slice=None, title="", subtitle="")

        return l_recovered_sentences_test
                   
    def mainFunc(self, plot=False, feedback=False, return_result=False):            
        # Definning parameters of stimulus (in a dictionary)
        d = {}
        d['act_time'] = 5
        d['pause'] = True
        d['suppl_pause_at_the_end'] = 1*d['act_time']
        d['initial_pause'] = True
        d['offset'] = False        
            
        [train_data_txt, test_data, sent_form_info_train, sent_form_info_test] = self.extract_data_io(path_file=self.corpusFile)    
        if self.verbose:
            print "train_data_txt : ", train_data_txt
            print "test_data : ", test_data
            print "sent_form_info_train : ", sent_form_info_train
            print "sent_form_info_test : ", sent_form_info_test
        train_corpus, train_meaning  = self.txt2corpus_and_meaning(train_txt=train_data_txt)
        
        
        if self.verbose:
            print "train_corpus : ", train_corpus
            print "train_meaning : ", train_meaning
            
        sdir = os.path.dirname(os.path.abspath(self.fileResult))
        print "sdir main func : ", sdir
        
        if self.smode =="test":
            shelf = shelve.open(sdir + '/shelf_prod.db', flag='r')
            flag = shelf.has_key("l_ocw_array_train")
            if flag:
                test_corpus = test_data

                self.imax_nr_actionrelation = shelf["imax_nr_actionrelation"]
                self.imax_nr_ocw = shelf["imax_nr_ocw"]

                l_recovered_sentences_test = self.test(test_corpus, sent_form_info_test, shelf)
                
                
                ## Writting sentences to output file
                #print " *** Writting to output file ... *** "
                l_final_sent_test = []
                for list_words in l_recovered_sentences_test:
                    l_final_sent_test.append(" ".join(list_words))
                    
        
                print "********************************************** "
                print " *** RECOGNIZED SENTENCES *** "
        
                for sent in l_final_sent_test:
                    print sent
        
                self.write_list_in_file(l_final_sent_test, file_path=self.fileResult)
            else:
                raise Exception, "The train was not launched"
                
            shelf.close()


        else:
            focus=[]
            for i in range (len(sent_form_info_train)):
                idx1=[a for a, x in enumerate(sent_form_info_train[i]) if x == "["]
                idx2=[b for b, x in enumerate(sent_form_info_train[i]) if x == "]"]
                l=[]
                for j in range (len(idx1)):
                    l.append(sent_form_info_train[i][idx1[j]+1:idx2[j]].split('-'))
                focus.append(l)

            self.imax_nr_ocw = 0
            self.imax_nr_actionrelation = 0
            
            for i in range (len(focus)): 
                itemp = 0
                temp_nr_actionrelation = len(focus[i]) 
                for j in range (len(focus[i])):
                    for k in range (len(focus[i][j])):
                        if focus[i][j][k] != '_':
                            itemp+=1
                    if itemp > self.imax_nr_ocw:
                        self.imax_nr_ocw = itemp
                if temp_nr_actionrelation > self.imax_nr_actionrelation:
                    self.imax_nr_actionrelation = temp_nr_actionrelation             
            

            shelf = shelve.open(sdir + '/shelf_prod.db', writeback=True)
            l_ocw_array_train, states_out_train, construction_words, internal_states_train, res, stim_mean_train, stim_sent_train, l_m_elt = \
                self.train(sent_form_info_train, train_meaning, train_corpus, d)

            shelf["l_ocw_array_train"] = l_ocw_array_train
            shelf["states_out_train"] = states_out_train
            shelf["construction_words"] = construction_words
            shelf["internal_states_train"] = internal_states_train
            shelf["res"] = res
            shelf["stim_mean_train"] = stim_mean_train
            shelf["stim_sent_train"] = stim_sent_train
            shelf["l_m_elt"] = l_m_elt
            shelf["imax_nr_actionrelation"]= self.imax_nr_actionrelation
            shelf["imax_nr_ocw"]= self.imax_nr_ocw
            shelf.close()
        print ""

if __name__ == '__main__':
    import os, sys

    sdir = os.path.dirname(os.path.abspath(__file__))
    print "sdir : ", sdir 
    corpusFile= sys.argv[1] # "corpus_narratif_scenario3_XPAOR.txt"
    fileResult = sys.argv[2]
    sMode =  sys.argv[3]   
    closed_class_words = sys.argv[4].split(',') #   ['after', 'than', 'before', 'to', 'the', 'slowly', 'quickly', 'with', 'that', 'for', 'a', 'an', 'this', 'of', 'and', 'while', 'when'] 
    l_elt_pred= sys.argv[5].split(',')     #['P','A','O','R','V']
    iNbNeurons = int(sys.argv[6]) #600
    
    prodSentences = Production(corpusFile, fileResult, sMode, closed_class_words, l_elt_pred, iNbNeurons, False)

#    sdir = os.path.dirname(os.path.abspath(__file__))
#    corpusFile = sdir + "/Corpus/corpus.txt"
#    fileResult = sdir + "/Corpus/output.txt"
#
#    closed_class_words = ['the', 'a', 'and', 'to','that']
#    l_elt_pred = ['P','A','O','R','V','W']
#    iNbNeurons = 1000
#    lMode = ["train", "test"]
#    for sMode in lMode:    
#        prodSentences = Production(corpusFile, fileResult, sMode, closed_class_words, l_elt_pred, iNbNeurons, False)


    print "*********END OF PROGRAM********"
