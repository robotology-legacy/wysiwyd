#! /usr/bin/env python
#encoding:UTF-8
'''
Created on 28 october 2011

@author: xavier HINAUT
xavier.hinaut@inserm.fr
'''

from __future__ import division

def remove_string_from_list_words(sentence, string):
    """
    Remove the given string from a list of words.
    
    Example :
    Inputs:
        sentence = ['<sil>', 'fill', 'may', 'have', 'a', 'ball', 'and', 'teeth', 'one', 'minute', 'common', 'next', '<sil>', 'azer', '<sil>', '<sil>', 'azer', '<sil>']
        string = '<sil>'
    (after processing):
        sentence = ['fill', 'may', 'have', 'a', 'ball', 'and', 'teeth', 'one', 'minute', 'common', 'next', 'azer', 'azer']
        
    """
    while 1:
        try:
            sentence.remove(string)
        except Exception:
#            print "removing finished"
            break
#        print "removing"
    return sentence

def l_has_words(l_ref, l_test):
    """
    makes two lists, one with the elements that are in l_test and l_ref, and one with the words present in l_test but not in l_ref
    """
    in_list = []
    not_in_list = []
    for w in l_test:
        if w in l_ref:
            in_list.append(w)
        else:
            not_in_list.append(w)
    return (in_list, not_in_list)

def string_to_l_word_sentence(l_string_sent):

    """
    Inputs:
        l_string_sent: list of sentences in the form of a list of strings
    Outputs:
        lws: list of sentences in the form of a list of list of words
    """
#    lss = copy.deepcopy(l_string_sent)
    lss = l_string_sent
    lws = []
    for ss in lss:
        new_s = ss.split(' ')
        lws.append(new_s)
    return lws

def extract_words_from_sent(sentences, l_words=[]):
    """
    inputs:
        - sentences: list of sentences in the form of a list of list of word (l_word_sentence)
        - l_words: a list of words
        
    output:
        - l_words: a list of words
    """
    for s in sentences:
        for w in s:
            if w not in l_words:
                l_words.append(w)
    return l_words

def get_sub_dic_from(d_words, l_words):            
    """
    Gets the sub-dictionary taking d_words
    
    inputs:
        - d_words: each key is a word, the corresponding value is not processed (it remain unchanged).
        - l_words: a list of words
        
    output:
        - new_d: new dictionary containing the words of sentences
        - words_not_in_dic: list containing the list of words that were not found in the dictionary
    """
    new_d = {}
    words_not_in_dic = []
    for w in l_words:
        if d_words.has_key(w):
            new_d[w] = d_words[w]
        else:
            words_not_in_dic.append(w)
    
    if words_not_in_dic!=[]:
        print "Some of the words were not found in the dictionary."
        print "Words not in dictionary: "+str(words_not_in_dic)
#        raise Warning, "Some of the words were not found in the dictionary."
    return (new_d, words_not_in_dic)

def empty_string(s):
    if s=='':
        return True
    elif s==' ':
        return True
#    elif s==" ":
#        return True
    else:
        return False
    
def count_nr_words(l_data):
    """
    Counts the number of words for each sentence, and return them as a list.
    
    inputs:
        - l_data: is a list of sentences. Each sentence is a list of words.
    """
    l_res = []
    for s in l_data:
        l_res.append(len(s))
    return l_res

if __name__=='__main__':
    def test_2012_01_04():
        l = [['the','N-block','V-push','the','N-cylinder', '.'], #0 -- #15
              ['the','N-cylinder','was','V-push','by','the','N-block', '.'], #1
              ['the','N-block','V-give','the','N-cylinder','to','the','N-moon', '.'], #2
              ['the','N-cylinder','was','V-give','to','the','N-moon','by','the','N-block', '.'], #3
              ['the','N-block','V-give','the','N-moon','the','N-cylinder', '.'], #4
              ['the','N-block','that','V-push','the','N-cylinder','V-touch','the','N-moon', '.'], #5 #20
              ['the','N-block','was','V-push','by','the','N-moon','that','V-touch','the','N-cylinder', '.'], #6
              ['the','N-moon','that','V-push','the','N-cylinder','was','V-touch','by','the','N-block', '.'], #7
              ['the','N-cylinder','V-touch','the','N-moon','that','V-push','the','N-block', '.'], #8
              ['the','N-block','that','was','V-push','by','the','N-moon','V-touch','the','N-cylinder', '.'], #9
              ['the','N-cylinder','was','V-touch','by','the','N-block','that','was','V-push','by','the','N-moon', '.'], #10 #25
              ['the','N-moon','that','was','V-touch','by','the','N-cylinder','was','V-push','by','the','N-block', '.'], #11
              ['the','N-moon','V-take','the','N-cylinder','that','was','V-touch','by','the','N-block', '.'], #12 #UNIQUE SENTENCE WITH V-TAKE
              ['the','N-cylinder','was','V-give','to','the','N-moon','by','the','N-block','that','V-touch','the','N-moon', '.'], #13
              ['the','N-block','that','V-touch','the','N-moon','was','V-give','to','the','N-cylinder','by','the','N-dog', '.'], #14
              ['the','N-cat','V-give','the','N-dog','to','the','N-cylinder','that','V-push','the','N-block', '.'], #15 #30
              ['the','N-cat','was','V-give','from','the','N-dog','to','the','N-block','that','V-push','the','N-cylinder', '.'], #16
              ['the','N-cylinder','that','was','V-push','by','the','N-block','V-give','the','N-cat','to','the','N-dog', '.'], #17
              ['the','N-block','V-give','the','N-moon','to','the','N-cylinder','that','was','V-touch','by','the','N-cat', '.'], #18
              ['the','N-cat','that','V-give','the','N-block','to','the','N-moon','V-push','the','N-cylinder', '.'], #19
              ['the','N-block','was','V-push','by','the','N-moon','that','V-give','the','N-cat','to','the','N-cylinder', '.'], #20 #35
              ['the','N-block','V-push','the','N-moon','that','V-give','the','N-cat','to','the','N-cylinder', '.'], #21
              ['the','N-block','that','V-give','the','N-moon','to','the','N-cat','was','V-push','by','the','N-cylinder', '.'], #22
              ['the','N-block','that','was','V-give','to','the','N-moon','by','the','N-cat','V-push','the','N-cylinder', '.'], #23
              ['the','N-block','V-push','the','N-moon','that','was','V-give','by','the','N-cat','to','the','N-cylinder', '.'], #24
              ['the','N-block','that','V-push','the','N-moon','V-give','the','N-cat','to','the','N-cylinder', '.'], #25 --#40
              ## object-relative sentences (added on Friday, 23th of September)
              ['the','N-dog','that','the','N-cat','V-hit','V-push','the','N-block', '.'], #0 -- #41 object relative, N1-O1 N1-A2 N2-A1 N3-O2
              ['the','N-dog','that','the','N-cat','V-hit','was','V-push','by','the','N-block', '.'], #1 -- #42 N1-01 N1-02 N2-A1 N3-A2
              ['the','N-dog','that','the','N-cat','V-hit','V-give','the','N-block','to','the','N-moon', '.'], #2 -- #43 N1-O1 N1-A2 N2-A2 N3-O2 N4-R2
              ['the','N-dog','that','the','N-cat','V-hit','V-give','the','N-block','the','N-moon', '.'], #3 -- #44 N1-O1 N1-A2 N2-A2 N3-R2 N4-02
              ]
        print "l_inputs", extract_words_from_sent(sentences=l)
        
    test_2012_01_04()