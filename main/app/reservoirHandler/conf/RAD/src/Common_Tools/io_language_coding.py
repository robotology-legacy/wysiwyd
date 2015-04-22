# -*- coding: utf-8 -*-
"""
Created on 12 déc. 2011

@author: Xavier HINAUT
xavier.hinaut #/at\# inserm.fr
"""

import mdp

import language as CTlang


# import Common_Tools.language as CTlang
def _stim_gen(l_input, sentence, act_time, suppl_pause_at_the_end, full_time, pause=True, initial_pause=True, offset=None):
    """
    Returns the stimulus input corresponding to the sentence selected.
        See method 'get_info_stim()' for more details on the input sentences.
    Important :  the numpy array 'stim' returned has to be tranposed to be used in a reservoir
    
    Inputs: 
        - l_input: list of all possible words given in input. The length of this list gives the input dimension.
        - full_time: whole number of time step of a stimulus (precedently it didn't include the initial_pause, now it include it)
        - offset: represents the difference between the maximum number of words in the data and the number of word of a given sentence.
            when taking into account the offset, offset has to be multiplied by 'act_time' (1 or 2 times depending if there 'pause' is false or true)
        
    If an offset is set, the '_' are not taken into account (there are not escape like spaces, but are taken like real words). Just a question of saving execution time.
    
    Variables:
        - sentence[i]: i+1-th word of the current sentence
        - l_input.index(sentence[i]): represents the index of the word 'sentence[i]' in the input stimulus
    """
    # Initializations
# TODO: suppress this addition of 2 act_time in case supplementary pause is None
    if suppl_pause_at_the_end is None:
        suppl_pause_at_the_end = 2 * act_time  # supplementary pause at the end of the sentence
    if initial_pause is True:
        j = 1
    else:
        j = 0
#    len_stim_in_words = len(sentence)
    if pause == True:
        # j = 1 #previously when initial_pause did not exist
        time_pause = 1  # duration of a pause between the presentation of 2 words, in nb of 'act_time'. Set this value to 0 if you don't want to have pauses.
#        if full_time is None:
#            print "_stim_gen: Evaluating the full time of the stimulus."
#            mult = len_stim_in_words
#            #full_time = 2*act_time*mult
#            full_time = 2*act_time*mult + suppl_pause_at_the_end
    else:
        # j = 0 #previously when initial_pause did not exist
        time_pause = 0
#        if full_time is None:
#            print "_stim_gen: Evaluating the full time of the stimulus."
#            mult = len_stim_in_words
#            #full_time = act_time*mult
#            full_time = act_time*mult + suppl_pause_at_the_end
    stim = mdp.numx.zeros((len(l_input), full_time))  # stimulus (returned value)
    # Generating the stimulus protocol while processing the sentence
#    for i in range(len_stim_in_words):
    if offset is None:
        for i in range(len(sentence)):
            word = sentence[i]
            if word == '_':
                pass  # an underscore is for an absence of stimulus, so we do nothing
            else:
    #            print "word="+word
                idx = l_input.index(word)
#                stim[idx][act_time*j:act_time*(j+1)] = mdp.numx.ones((1,act_time)) 
                stim[idx, act_time * j:act_time * (j + 1)] = mdp.numx.ones((1, act_time)) 
            j = j + 1 + time_pause  # each stimulus is separated by a pause
    else:
        j = j + offset * (1 * (pause == False) + 2 * (pause == True))
#        print "offset", offset
#        print "act_time",act_time
#        print "sentence:", sentence
#        print "j", j
#        print "len(l_input)", len(l_input)
#        print "l_input", l_input
#        print "full_time", full_time
        for i in range(len(sentence)):
#            print "i",i
#            print "sentence[i]", sentence[i]
#            print "act_time*j:act_time*(j+1)", str(act_time*j)+':'+str(act_time*(j+1))
            if sentence[i] == '_':
                pass  # an underscore is for an absence of stimulus, so we do nothing
            else:
                stim[l_input.index(sentence[i]), act_time * j:act_time * (j + 1)] = mdp.numx.ones((1, act_time)) 
            j = j + 1 + time_pause  # each stimulus is separated by a pause
    return stim

def _stim_gen_T(*args, **kwargs):
    return _stim_gen(*args, **kwargs).T

def _output_gen_start_act_time(l_output, AOR, act_time, full_time, pause, suppl_pause_at_the_end, nr_words, initial_pause=True, start=None, offset=None, verbose=False):
    """
    Returns the teacher outputs signal corresponding to the AOR (AOR: Agent-Object-Recipient) output selected corresponding to the same index sentence of get_info_stim.
        See method 'get_info_teacher_output()' for more details on the sentences.
    Important :  the numpy array 'teach' returned has to be transposed to be used in a reservoir
    
    The output teacher is forced to one since a certain number of 'act_time' indicated by 'start'
    
    Modification of method _output_gen(l_output, AOR, act_time, full_time, pause, suppl_pause_at_the_end, initial_pause=True):
        in order to be able to set an arbitrary moment where to begin the output.
        if 'start' is set to 'end', this means that the output will be asked since the beginning of the last "element" of the sentence (an element could be a word or a sign of punctuation like a dot).
    
    Input:
        - AOR: output desired coded in the AOR fashion: it corresponds to the current line in 'l_teacher' obtained with the method get_info_teacher_output()
        - full_time: whole number of time step of a teacher output signal (precedently it didn't include the initial_pause, now it include it)
        - nr_words: indicates the number of words or word-like (dot or other type of ponctuation)
        - start: if it's a number it indicates the position of the word where will start the teacher output signal (this number starts from 1, not from 0)
            if a decimal part is present, it indicates the relative position during the stimulus: e.g. 1.5 indicates that the signal will begin at the middle of the 1st word.
                If a fractional part exist it has to be taken into account, so it cannot be zero, that's why we take the upper closer integer.
            if the number is negative, this means that we consider the starting of the signal during the pause that is just after the word stimulus (there is just a warning in case there is no pause defined)
                !!! -1.25 indicates that the signal will begin at the first quarter of the pause that is just after the 1st word: the decimal part is interpreted separately from negativity (I.e. the fact that the number is negative)
        - offset: represents the difference between the maximum number of words in the data and the number of word of a given sentence.    
            when taking into account the offset, offset has to be multiplied by 'act_time' (1 or 2 times depending if there 'pause' is false or true)    
    """
    # Initializations
    if initial_pause is True:
        j = 1
    else:
        j = 0
    if start is None:
        st = 0
        fr = 0
    else:
        if start == 'end':  # checks if start is not a number
            st = int(nr_words) - 1
            fr = 0
        elif -1 < start < 1:  # if it is a number, check if it has a correct value
            raise Exception, "argument 'start' cannot be between -1 and 1 (superior to -1 and inferior to 1). "
        else:
#            st = int(mdp.numx.fabs(start))-1
            (fr, st) = mdp.numx.modf(mdp.numx.fabs(start))  # math.modf(x) returns the fractional and the integer part of x
            if verbose:
                print "nr_words:", nr_words
                print "st:", st
            if st > nr_words:
                raise Exception, "The start point indicated for the output teacher is too large for the data: 'start' exceeded the total number of words. start=" + str(start) + " ; nr_words=" + str(nr_words)
            st = int(st - 1)  # start begins at 1 not at 0 like the index
            fr = int(mdp.numx.ceil(act_time * fr))  # take the smallest integer value greater than or equal to (act_time*pc). If a fractional part exist it has to be taken into account, so it cannot be zero, that's why we take the upper closer integer.
    if pause == True:
        st = int(st * 2)
        if start < 0:  # this is False if start equals to 'end'
            # if start is negative (non positive), the signal has to start during the pause
            st = int(st + 1)
    else:
        if start < 0: 
            raise Warning, "argument 'start' is negative and 'pause' is not set to True. Information ignored, output teacher signal will start during the word and not during pause (because there is no pause)."
    teach = mdp.numx.zeros((len(l_output), full_time))  # stimulus (returned value)
    if offset is None:
        if (act_time * (j + st) + fr) >= full_time:
            raise Warning, "The output teacher is beginning to late: consequently the teacher output will be all zeros. act_time*(j+st)+fr)=" + str(act_time * (j + st) + fr) + " ; full_time=" + str(full_time)
        for i in range(len(AOR)):
            # TODO: collapse these 3 lines in one line, like when offset is used
            out_elt = AOR[i]  # one output element information
            idx = l_output.index(out_elt)
            # teach[idx][act_time*j:full_time] = mdp.numx.ones((1,full_time-(act_time*j)))
#            teach[idx][act_time*(j+st)+fr:full_time] = mdp.numx.ones((1,full_time-(act_time*(j+st)+fr)))
            teach[idx, act_time * (j + st) + fr:full_time] = mdp.numx.ones((1, full_time - (act_time * (j + st) + fr)))
    else:
        off = offset * (1 * (pause == False) + 2 * (pause == True))
        if (act_time * (j + st + off) + fr) >= full_time:
            raise Warning, "The output teacher is beginning to late: consequently the teacher output will be all zeros. act_time*(j+st+off)+fr)=" + str(act_time * (j + st + off) + fr) + " ; full_time=" + str(full_time)
        for i in range(len(AOR)):
#            out_elt = AOR[i] # one output element information "
#            idx = l_output.index(out_elt)
            # # Testing When offset if used we do the 3 operations in one line
#            teach[l_output.index(AOR[i])][act_time*(j+st+off)+fr:full_time] = mdp.numx.ones((1,full_time-(act_time*(j+st+off)+fr)))
            teach[l_output.index(AOR[i]), act_time * (j + st + off) + fr:full_time] = mdp.numx.ones((1, full_time - (act_time * (j + st + off) + fr)))
    if verbose:
        if offset is None:
            print "nr_words:", nr_words, " _ start:", start, " _ st:", st , " _ fr:", fr , " _ j:", j
            print "j+st=", str(j + st)
            print "act_time*(j+st)+fr: ", act_time * (j + st) + fr, " _ full_time:", full_time
        else:
            print "nr_words:", nr_words, " _ start:", start, " _ offset:", offset, " _ st:", st , " _ fr:", fr , " _ j:", j, " _ off:", off
            print "j+st+off=", str(j + st + off)
            print "act_time*(j+st+off)+fr: ", act_time * (j + st + off) + fr, " _ full_time:", full_time
        print "ex of teacher output:", teach[l_output.index(AOR[i])], '\n'
    return teach

def _output_gen_start_act_time_T(*args, **kwargs):
    return _output_gen_start_act_time(*args, **kwargs).T

def get_full_time(dp, mult):
    if dp['pause'] == True:
        if dp['initial_pause']:
            full_time = 2 * dp['act_time'] * mult + dp['suppl_pause_at_the_end'] + dp['act_time']  # full time of stimulus
        else:
            full_time = 2 * dp['act_time'] * mult + dp['suppl_pause_at_the_end']  # full time of stimulus
    else:
        if dp['initial_pause']:
            full_time = dp['act_time'] * mult + dp['suppl_pause_at_the_end'] + dp['act_time']  # full time of stimulus
        else:
            full_time = dp['act_time'] * mult + dp['suppl_pause_at_the_end']  # full time of stimulus
    return full_time

def get_full_time_nodic(act_time, mult, pause=False, initial_pause=False, suppl_pause_at_the_end=0):
    nr_step_init = act_time * (0 * (initial_pause == False) + 1 * (initial_pause == True))
    nr_step_sent = act_time * mult * (1 * (pause == False) + 2 * (pause == True))
    full_time = nr_step_init + nr_step_sent + suppl_pause_at_the_end
    return full_time

def generate_stim_input_nodic(l_data, act_time=1, subset=None, l_input=None,
                              l_nr_word=None, mult=None, full_time=None,
                              with_offset=True, pause=False, initial_pause=False,
                              suppl_pause_at_the_end=0, verbose=False):
    """
    Inputs:
        - l_data: list of list of words: list of sentences, sentences are in the form of a list of words
        - mult: usually the maximum number of words in the sentence, but could be bigger if user wants.
    
    Outputs:
        - l_offset: The offset represents the difference between the maximum number of words in the data and the number of word of a given sentence.
    """
    if subset is None:
        subset = range(len(l_data))
    if l_input is None:
        l_input = CTlang.extract_words_from_sent(sentences=l_data)
    if l_nr_word is None:
        l_nr_word = CTlang.count_nr_words(l_data=l_data)
    if mult is None:
        mult = max(l_nr_word)
    if full_time is None:
        full_time = get_full_time_nodic(act_time=act_time, mult=mult, pause=pause, initial_pause=initial_pause, suppl_pause_at_the_end=suppl_pause_at_the_end)
    # check if subset is too large for the data
    if len(subset) > len(l_data):
        s = "The length of the subset is too large. Input data has a lower size than the subset: the length of the subset is " + str(len(subset)) + " but the length of the input data is " + str(len(l_data)) + "."
        raise Exception, s
    # # check number of words
    if l_nr_word != CTlang.count_nr_words(l_data=l_data):
        raise Exception, "d_in['l_nr_word'] does not contain the correct number of words."
    stim_data = len(subset) * [mdp.numx.zeros((len(l_input), full_time))]
    if with_offset:
        l_offset = [mult - x for x in l_nr_word]  # The offset represents the difference between the maximum number of words in the data and the number of word of a given sentence.
    else:
        l_offset = [None] * len(l_nr_word)
    idx_stim = 0
    for i in subset:
        stim_data[idx_stim] = _stim_gen_T(l_input=l_input, sentence=l_data[i], act_time=act_time, full_time=full_time, pause=pause, suppl_pause_at_the_end=suppl_pause_at_the_end, initial_pause=initial_pause, offset=l_offset[i])
        idx_stim = idx_stim + 1
    return (stim_data, l_offset)

def generate_teacher_output(dp, d_in, d_out, verbose=False):
    """ 
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
    
    teacher_output = len(dp['subset']) * [mdp.numx.zeros((len(d_out['l_output']), d_in['full_time']))]
#    l_offset = [mult-x for x in l_nr_word] #The offset represents the difference between the maximum number of words in the data and the number of word of a given sentence.
    if dp['offset']:
        l_offset = d_in['l_offset']
    else:
        l_offset = [None] * len(d_in['l_nr_word'])
    if d_out.has_key('l_teacher'):
        pass
    else:
        d_out['l_teacher'] = [[x] for x in d_out['out_class']]
    idx_teach = 0
    for i in dp['subset']:
#        nr_words = len(l_data[i])
        nr_words = d_in['l_nr_word'][i]
#        teacher_output[idx_teach] = _output_gen_start_act_time(l_output=l_output, AOR=l_teacher[i], act_time=act_time, full_time= d_in['full_time'], pause=pause, suppl_pause_at_the_end=suppl_pause_at_the_end, nr_words=nr_words, start=start,  initial_pause=initial_pause, offset=l_offset[i]).T
        teacher_output[idx_teach] = _output_gen_start_act_time_T(l_output=d_out['l_output'], AOR=d_out['l_teacher'][i],
                                    act_time=dp['act_time'], full_time=d_in['full_time'], pause=dp['pause'],
                                    suppl_pause_at_the_end=dp['suppl_pause_at_the_end'], nr_words=nr_words,
                                    start=dp['start_teacher'], initial_pause=dp['initial_pause'],
                                    offset=d_in['l_offset'][i])
        idx_teach = idx_teach + 1
        
    return teacher_output

if __name__ == '__main__':
    pass
