#! /usr/bin/env python
#encoding:UTF-8
'''
Created on 17 november 2011

@author: xavier HINAUT
xavier.hinaut@inserm.fr

'''

from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot
import pylab as pl
import mdp


def ___info_on_color_map():
    pass
    import matplotlib.cm as cm
#    (labels, lab_tick) = Language.stim_eng_gram_const.get_label_sentence(subset=subset, noun_as_1_dim=noun_as_1_dim, verb_as_1_dim=verb_as_1_dim, pause=pause)
#    (x, l_input, x, x) = Language.stim_eng_gram_const.get_info_stim(noun_as_1_dim=noun_as_1_dim, verb_as_1_dim=verb_as_1_dim)
#    print "l_input in main method", l_input
#    # creating a vector 'c' countaining the differents colors for de legend
#    c = [None]*len(l_input)
#    for i in range(len(l_input)):
#        c[i] = cm.hsv((i*1.0)/len(l_input),1) #choose color in colormap 'hsv'
#    # plotting the outputs
#    ppRO = PdfPages('RES_TEMP/'+root_file_name+'_'+str(subtitleReservoir)+'_readout_states.pdf') # Internal Outputs are the outputs of the neurons in the reservoir
#    #ppRO = PdfPages('RES_TEMP/datatest_sent_gen_'+str(subtitleReservoir)+'_readout_seq'+str(subset[idx_out])+'.pdf') # for separated files
#    for i in range(len(states_out)):
#        pylab.figure()
#        for j in range(len(l_input)):
#            pylab.plot(states_out[i][:,j],color=c[j])
#        #pylab.plot(states_out[i],color=c) #pylab.plot(states_out[i]) #
#        pylab.legend(l_input[:], loc='upper right')
#        pylab.title("Output for sentence "+str(i)+" (desired output is: '"+" ".join(labels[i])+"')")
#        ppRO.savefig()
#    ppRO.close()


def check_for_unity_dim(dims, param2expl, mat=None):
    """
    Check if some dimensions are equal to 1.
    Returns dims, param2expl and param_ignored without the corresponding
        elements with dimensions equal to 1.
    """
    new_dims = []
    new_param = param2expl[:]
    param_ignored = []
    for i in range(len(dims)):
        if dims[i]==1:
            # if the current dimension is unity, remove the corresponding parameter from the final parameter list
            new_param.remove(param2expl[i])
            param_ignored.append(param2expl[i])
        else:
            # if current dimension is not unity, add it to final dimension list
            new_dims.append(dims[i])
    if mat is not None:
        import copy
        mat_res = copy.deepcopy(mat)
        mat_res.resize(new_dims)
        return (new_dims, new_param, param_ignored, mat_res)
    else:
        return (new_dims, new_param, param_ignored)

def is_float(x):
    if type(x) is float:
        return True
    else:
        import numpy
        if type(x) is numpy.float64:
            print "numpy.float64 is the type of x"
            return True
        else:
            print "type of x is ", type(x)
            return False

def plot_array_in_file(root_file_name, array_, data_subset=None, titles_subset=None, plot_slice=None, title="", subtitle="", legend_=None):
    """
    
    inputs:
        - array_: is the array or matrix to plot
        - data_subset: correspond to the subset of the whole data that is treated. array_ is corresponds to this subset. /
            array_ and subset have to have the same length
        - plot_slice: slice determining the element of array_ that will be plotted.
    """
    if data_subset is None:
        data_subset = range(len(array_))
    if titles_subset is None:
        titles_subset = ['' for _ in range(len(data_subset))]
        nl_titles_sub = ''
    else:
        nl_titles_sub = '\n'
    if array_==[] or array_==mdp.numx.array([]):
#        print "Warning: array empty. Could not be plotted. Title:"+str(title)
        import warnings
        warnings.warn("Warning: array empty. Could not be plotted. Title:"+str(title))
        return
    if plot_slice is None:
        plot_slice = slice(0,len(data_subset))
    else:
        if (plot_slice.stop-1) > len(data_subset):
            print "len(data_subset)", len(data_subset)
            print "(plot_slice.stop-1)", (plot_slice.stop-1)
            raise Exception, "The last element of the slice is out of the subset."
        subtitle = subtitle+"_slice-"+str(plot_slice.start)+"-"+str(plot_slice.stop)+"-"+str(plot_slice.step)
    ppIS = PdfPages(str(root_file_name)+str(title)+'.pdf')
    
    for i in range(plot_slice.stop)[plot_slice]:
        pl.figure()
#        pl.suptitle(title+" "+str(titles_subset[i])+nl_titles_sub+" - seq "+str(data_subset[i])+"\n"+subtitle)
        pl.plot(array_[i])
        if legend_ is not None:
            pl.legend(legend_)
#        [xmin, xmax, ymin, ymax] = pl.axis()
#        pl.axis([xmin, xmax, -0.4, 1.200001])
        ppIS.savefig()
        pl.close() # Very important!!!! Do not forget this line when not showing images at the screen, but recording them in a file: this free the memory occupied by the figure (which can became huge when there is 100s of figures).
    ppIS.close()

def plot_mat(mat, title_, xticks_, yticks_, label_x, label_y, xlabel_, ylabel_, min=None, max=None, type_plot="matshow", format_string="%.3f"):
    def reformat_labels(lab):
        print "label",lab
        print "len(lab)",len(lab)
#        if len(lab)>8:
        if len(lab)>10:
#            q = len(lab)/8 + 1
#            q = int(mdp.numx.ceil(len(lab)/8.))
#            q = int(mdp.numx.floor(len(lab)/8.))
#            q = int(mdp.numx.ceil(len(lab)/10.))
            q = int(mdp.numx.floor(len(lab)/10.))
#            q = int(mdp.numx.ceil(len(lab)/6.))
            print "q:",q
            print "returned:", str([lab[i] for i in range(len(lab)) if i%q==0])
            return ([lab[i] for i in range(len(lab)) if i%q==0],q)
        else:
            return (lab, 1)
    if type_plot=="imshow":
        pl.figure()
        pl.imshow(mat, interpolation='nearest', vmin=min, vmax=max)
    else:
        pl.matshow(mat, vmin=min, vmax=max)
    pl.title(title_+'\n')
    pl.xlabel(xlabel_)
    pl.ylabel(ylabel_)
    if xticks_ is not None:
        pl.xticks = xticks_
    if yticks_ is not None:
        pl.yticks = yticks_
    a = matplotlib.pyplot.gca()
    locs, labels = pl.xticks()
    print "locs, labels",locs, labels
    ## format values on x and y, so that they can be plotted entirely
#    if label_x[0] is float:
    if is_float(label_x[0]):
        label_x = [format_string % x for x in label_x]
    if is_float(label_y[0]):
        label_y = [format_string % x for x in label_y]
    (label_x_r, qx) = reformat_labels(label_x)
    (label_y_r, qy) = reformat_labels(label_y)
    if type_plot=="matshow":
#        l_tmp = [-99999]
#        l_tmp.extend(label_x)
#        l_tmp = label_x
#        print "l_tmp", l_tmp
#        a.set_xticklabels(l_tmp, fontdict=None, minor=False) # set values on x ticks
        pl.xticks( mdp.numx.arange(0.,len(label_x),float(qx)), label_x_r )
#        l_tmp = [-99999]
#        l_tmp.extend(label_y_r)
#        print "l_tmp", l_tmp
#        a.set_yticklabels(l_tmp, fontdict=None, minor=False) # set values on y ticks
        pl.yticks( mdp.numx.arange(0.,len(label_y),float(qy)), label_y_r )
    else:
        a.set_xticklabels(label_x, fontdict=None, minor=False) # set values on x ticks
        a.set_yticklabels(label_y, fontdict=None, minor=False) # set values on y ticks
#    a.xaxis.set_tick()
#    a.xaxis.set_ticklabels()
    pl.colorbar()

def plot_err_matrix(mat, param, param_values, title="", filename_save_fig=None,
                    compute_std=False, format_string="%.3f", verbose=True):
    """
    Prints all error plots of all possible combinaisons of pairs of parameters
    
    Inputs:
        - mat: matrix of len(param) dimensions, contains all the error values
        - param: list of parameters with the dimensions in the same order than the matrix mat
        - param_values: dictionary containning the ranges of values taken by the parameters in param
        - compute_std: if True, compute the standard deviation instead of the mean
        
    TODO: add an argument param_order that indicates all the possible parameters and in which order they should appear -> this should resolve the problem of ambiguity when plotting, and other problem of plotting
    """
    if filename_save_fig is not None:
        ## define the file that will receive the images
        pp = PdfPages(str(filename_save_fig)+str(title)+'.pdf')
    ## compute min and max values in order to have the same colorbar and all plots
    import numpy
    global_min = numpy.min(mat)
    global_max = numpy.max(mat)
    if len(mat.shape)!=len(param):
        print "!!! Warning: the number of dimensions of the matrix is "+str(len(mat.shape)) \
            +" and the number of parameters is "+str(len(param))+"." \
            +"\n !!! There do not correspond. There might be a problem."
    if len(param)==1:
        # do a 1-dimension plot (simple plot)
        pl.figure()
        pl.plot(mat)
        pl.plot()
        local_min = numpy.min(mat)
        local_max = numpy.max(mat)
        subtitle = "\nmin=%.3f max=%.3f" %(local_min, local_max)
        pl.title(str(title)+subtitle)
        print "param", param
        print "param_values", param_values
        pl.xlabel(param_values[param[0]])
        if filename_save_fig is not None:
            pp.savefig()
            pl.close()
    import itertools
    for pair in itertools.combinations(param, 2):
        # set of parameters that would not be plotted for this pair,
            #it corresponds to the axis/parameters on which we are computing the mean,
            # we average on these parameters. The 2 axis not averaged are those in pair,
            # they will be the axis/parameters plot on the final graphic
        set_not_plot = set(param)-set(pair)
        if verbose:
            print "pair of parameters treated: "+str(pair)
            print str(set_not_plot)
        ## do the mean of the matrix for each dimension that is not plotted
        shape = list(mat.shape)
        m = mat
        print "m.shape",m.shape
        ## for each parameter (=axis) in set_not_plot,
            # we compute the mean on this axis (i.e. we average on this axis)
        for s in set_not_plot:
#            m = mdp.numx.squeeze(mdp.numx.mean(m, param.index(s)))
            print "param.index(s)", str(param.index(s))
            if compute_std:
            ## Compute std instead of the default mean
                m = mdp.numx.std(m, param.index(s))
            else:
                m = mdp.numx.mean(m, param.index(s))
            shape[param.index(s)] = 1
            print "m.shape before reshape",m.shape
            m.resize(shape)
            print "m.shape after reshape",m.shape
        m = mdp.numx.squeeze(m) # remove dimensions useless
        print "m.shape after final squeeze",m.shape
        if len(m.shape)!=2:
#            raise Warning, "The shape of the matrix to plot has not 2 dimensions. Can only plot matrices with 2 dimensions."
            print "!!! Warning: The shape of the matrix to plot has not 2 dimensions. Can only plot matrices with 2 dimensions."
        ## compute the min and max for this particular sub-matrix
        local_min = numpy.min(m)
        local_max = numpy.max(m)
        ## plot the matrix obtained
        subtitle = "\nmin=%.3f max=%.3f" %(local_min, local_max)
        plot_mat(m, title_=title+subtitle, xticks_=None, yticks_=None, label_x=param_values[pair[1]],
                 label_y=param_values[pair[0]], xlabel_=pair[1], ylabel_=pair[0],
                 min=global_min, max=global_max, format_string=format_string)
        if filename_save_fig is not None:
            pp.savefig()
            pl.close()
    if filename_save_fig is not None:
        pp.close()

def plot_err_matrix_with_not_averaged_param(mat, param, param_values, not_average_param, title="", filename_save_fig=None,
                    compute_std=False, format_string="%.3f", verbose=True):
    """
    Prints all error plots of all possible combinaisons of pairs of parameters
    
    Inputs:
        - mat: matrix of len(param) dimensions, contains all the error values
        - param: list of parameters with the dimensions in the same order than the matrix mat
                all parameters that are not in this list will be averaged for all plots
                parameters in not_average_param should be in param also
        - param_values: dictionary containning the ranges of values taken by the parameters in param
        - compute_std: if True, compute the standard deviation instead of the mean
        
    TODO: add an argument param_order that indicates all the possible parameters and in which order they should appear -> this should resolve the problem of ambiguity when plotting, and other problem of plotting
    """
    def plot_2d_mat(matrix_2d, suppl_subtitle=''):
        if len(matrix_2d.shape)!=2:
#            raise Warning, "The shape of the matrix to plot has not 2 dimensions. Can only plot matrices with 2 dimensions."
            print "!!! Warning: The shape of the matrix to plot has not 2 dimensions. Can only plot matrices with 2 dimensions."
        ## compute the min and max for this particular sub-matrix
        local_min = numpy.min(matrix_2d)
        local_max = numpy.max(matrix_2d)
        ## plot the matrix obtained
        subtitle = suppl_subtitle+"\nmin=%.3f max=%.3f" %(local_min, local_max)
        plot_mat(matrix_2d, title_=title+subtitle, xticks_=None, yticks_=None, label_x=param_values[pair[1]],
                 label_y=param_values[pair[0]], xlabel_=pair[1], ylabel_=pair[0],
                 min=global_min, max=global_max, format_string=format_string)
        if filename_save_fig is not None:
            pp.savefig()
            pl.close()
    
    if verbose:
        print 'param', param
        print 'param_values', param_values
        print 'not_average_param', not_average_param
    
    if filename_save_fig is not None:
        ## define the file that will receive the images
        pp = PdfPages(str(filename_save_fig)+str(title)+'.pdf')
    ## compute min and max values in order to have the same colorbar and all plots
    import numpy
    global_min = numpy.min(mat)
    global_max = numpy.max(mat)
    if len(mat.shape)!=len(param):
        print "!!! Warning: the number of dimensions of the matrix is "+str(len(mat.shape)) \
            +" and the number of parameters is "+str(len(param))+"." \
            +"\n !!! There do not correspond. There might be a problem."
    if len(param)==1:
        # do a 1-dimension plot (simple plot)
        pl.figure()
        pl.plot(mat)
        pl.plot()
        local_min = numpy.min(mat)
        local_max = numpy.max(mat)
        subtitle = "\nmin=%.3f max=%.3f" %(local_min, local_max)
        pl.title(str(title)+subtitle)
        print "param", param
        print "param_values", param_values
        pl.xlabel(param_values[param[0]])
        if filename_save_fig is not None:
            pp.savefig()
            pl.close()
    import itertools
    for pair in itertools.combinations(param, 2):
        # set of parameters that would not be plotted for this pair,
            #it corresponds to the axis/parameters on which we are computing the mean,
            # we average on these parameters. The 2 axis not averaged are those in pair,
            # they will be the axis/parameters plot on the final graphic
        set_not_plot = set(param)-set(pair)
        if verbose:
            print "pair of parameters treated: "+str(pair)
            print str(set_not_plot)
        ## do the mean of the matrix for each dimension that is not plotted
        shape = list(mat.shape)
        m = mat
        print "m.shape",m.shape
        ## for each parameter (=axis) in set_not_plot,
            # we compute the mean on this axis (i.e. we average on this axis)
        multi_plot = False
        for s in set_not_plot:
            if s in not_average_param:
                multi_plot = True
            else:
    #            m = mdp.numx.squeeze(mdp.numx.mean(m, param.index(s)))
                print "param.index(s)", str(param.index(s))
                if compute_std:
                ## Compute std instead of the default mean
                    m = mdp.numx.std(m, param.index(s))
                else:
                    m = mdp.numx.mean(m, param.index(s))
                shape[param.index(s)] = 1
                print "m.shape before reshape",m.shape
                m.resize(shape)
                print "m.shape after reshape",m.shape
        m = mdp.numx.squeeze(m) # remove dimensions useless
        print "m.shape after final squeeze",m.shape
        if multi_plot:
            for n_avg_p in not_average_param:
#                for curr_value_p in param_values[n_avg_p]:
                for curr_idx_value_p in range(len(param_values[n_avg_p])):
                    # !!!: in this version of the code,
                    #     the parameters in not_average_param list have to be int the param list
                    if len(not_average_param)>1:
                        raise Exception, "NOT DONE: cannot have more than one parameter in not_average_param."\
                            + "\nTODO: instead of doing a simple 'for', do a map of all the values for all parameters in not_average_param."
#                    sup_subttl = "\nfor "+n_avg_p+"="+str(curr_value_p)
                    sup_subttl = "\nfor "+n_avg_p+"="+str(param_values[n_avg_p][curr_idx_value_p])
                    # get index in the parameter we are varying in the list of parameters param
                    idx_n_avg_p = param.index(n_avg_p)
                    sl = [] # create list of slices
                    for ip in range(len(param)):
                        if ip==idx_n_avg_p:
#                            sl.append(curr_value_p)
#                            sl.append(param_values[n_avg_p].index(curr_value_p))
                            sl.append(curr_idx_value_p)
                        else:
                            sl.append(slice(None))
                    if verbose:
                        print "not average for this parameter:", n_avg_p
                        print "curr_value_ parameter", param_values[n_avg_p][curr_idx_value_p] #curr_value_p
                        print "global slice for matrix:", sl
                    # plot matrix with the created slice
                    plot_2d_mat(m[sl],suppl_subtitle=sup_subttl)
        else:
            sup_subttl = "\navg on "+str(list(set_not_plot))
            plot_2d_mat(m,suppl_subtitle=sup_subttl)
    if filename_save_fig is not None:
        pp.close()


if __name__=='__main__':
#    def check_for_unity_dim(dims, param):
#        """
#        Check if some dimensions are equal to 1.
#        Returns dims and param without the corresponding elements with dimensions equal to 1.
#        """
#        new_dims = []
#        new_param = param[:]
#        param_ignored = []
#        for i in range(len(dims)):
#            if dims[i]==1:
#                # if the current dimension is unity, remove the corresponding parameter from the final parameter list
#                new_param.remove(param[i])
#                param_ignored.append(param[i])
#            else:
#                # if current dimension is not unity, add it to final dimension list
#                new_dims.append(dims[i])
#        return (new_dims, new_param, param_ignored)
                
    
    def extract_dims_from_param_dic(param, d_param):
        dims = []
        for k in param:
            print str(k)+" length:"+str(len(d_param[k]))
            dims.append(len(d_param[k]))
        return dims
    
    def extract_mat_from_list(list_, dimensions):
        mat = mdp.numx.array(list_)
        mat.resize(dimensions)
        return mat
    
    def extract_mat_from_list_4dim(list_, dimensions, init_meth=mdp.numx.zeros):
#       mat = init_meth((5,6,4,3))
        mat = init_meth(dimensions)
        l_idx = []
        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                for k in range(dimensions[2]):
                    for l in range(dimensions[3]):
#                            index = i*6*4*3+j*4*3+k*3+l
                        index = i*dimensions[1]*dimensions[2]*dimensions[3] \
                            +j*dimensions[2]*dimensions[3] \
                            +k*dimensions[3]+l
                        l_idx.append(index)
                        print "index="+str(index)
                        mat[i,j,k,l] = list_[index]
        return mat
    
    def example_err_plot_from_list_err():
        def extract_used_dimensions_from_incomplete_matrix(list_, full_dims, verbose=True):
            if verbose:
                print "initial dimensions:"+str(full_dims)
            dims = full_dims[:]
            dims.reverse()
            non_null_elt = len(list_)
            used_dim = 0
            q = non_null_elt # will receive the number of dimensions of the last dimension we will iterate on
            for d in dims: #we divide the total number of non null elements by\
                # each dimension, until q reach 0
                q_tmp = q/d
                if q_tmp==0:
                    break
                else:
                    q = q_tmp
                    used_dim = used_dim + 1
            final_dims = []
            full_dims_rev = full_dims[:]
            full_dims_rev.reverse()
#            idx = range(-1, -len(full_dims), -1)
            for i in range(len(full_dims)):
                # for each dimension in initial dimensions list, we iterate beginning from the end
                if i<used_dim: # if list is in 
                    final_dims.append(full_dims_rev[i])
                elif i==used_dim:
                    final_dims.append(q)
                else:
                    final_dims.append(1)
            final_dims.reverse()
            if verbose:
                print "final dimensions: "+str(final_dims)
            return final_dims
                    
        def check_for_dimensions_of_param(param, sub_dims):
            idx = 0
            for p in param:
                if len(d_param[p])>sub_dims[idx]:
                    raise Exception, "The range of parameter "+str(p)+" have to be redefined to a length of "+str(sub_dims[idx])
                else:
                    idx = idx + 1         
        
        param = ['at', 'sr', 'tau', 'is'] #['spectral_radius', 'leak_rate', 'input_scaling', 'instance']
#        d_param = {'spectral_radius':sr_range,
#                   'leak_rate':leak_range,
#                   'input_scaling':is_range,
#                   'instance':instance_range}
        d_param = {'at': range(1,6,1),
                   'sr': mdp.numx.arange(0.1,5.111,1), 
                   'tau': range(1,5,1),
                   'is': mdp.numx.arange(0.5,3,1)}
        
        dims = extract_dims_from_param_dic(param, d_param)
        
        import Parse_file.io_std_files
        list_ = Parse_file.io_std_files.load_file(
                                                  full_file_path="/home/xavier/Work/Prog/Oger/workspace/XavReservoir/RES_TEMP/cv_timit_fabian_1500___all_error",
                                                  option='rb')
        sub_dims = extract_used_dimensions_from_incomplete_matrix(list_, full_dims=dims)
        check_for_dimensions_of_param(param, sub_dims)
        matrix = extract_mat_from_list_4dim(list_, dimensions=sub_dims)
        print "matrix[0,0,0,:]", matrix[0,0,0,:]
        
        plot_err_matrix(mat=matrix, param=param, param_values=d_param, title="Mean errors",
                        filename_save_fig="../../RES_TEMP/cv-explor_timit_fabian_1500")
#        pl.show()

    def plot_sim_2011_11_22soir_fusion_sim():
        import Parse_file.io_std_files
        param = ['at', 'sr', 'tau', 'is']
        d_param = {'at': range(1,2,1),
           'sr': mdp.numx.arange(0.1,1,0.2),
           'tau': range(1,10,1),
           'is': mdp.numx.arange(0.5,2,0.5)}
        dims = extract_dims_from_param_dic(param, d_param)
        print "dims",dims
    #            (dims, param, param_ignored) = check_for_unity_dim(dims, param)
        list_sim = Parse_file.io_std_files.load_file(
                                          full_file_path="/home/xavier/Work/Prog/Oger/workspace/XavReservoir/RES_TEMP/cv_timit_fabian_1500___all_mean_error_sim1",
                                          option='rb')
        mat_sim1 = extract_mat_from_list(list_sim, dimensions=dims)
        print "mat_sim1.shape", mat_sim1.shape
        dims_sim1 = dims
        
        d_param = {'at': range(1,2,1),
           'sr': mdp.numx.arange(1,2,0.2),
           'tau': range(1,10,1),
           'is': mdp.numx.arange(0.5,2,0.5)}
        dims = extract_dims_from_param_dic(param, d_param)
        print "dims",dims
        list_sim = Parse_file.io_std_files.load_file(
                                          full_file_path="/home/xavier/Work/Prog/Oger/workspace/XavReservoir/RES_TEMP/cv_timit_fabian_1500___all_mean_error_sim2",
                                          option='rb')
        mat_sim2 = extract_mat_from_list(list_sim, dimensions=dims)
        print "mat_sim2.shape", mat_sim2.shape
        dims_sim2 = dims
        
        mat_sim12 = mdp.numx.concatenate((mat_sim1, mat_sim2), axis=1)
        print "mat_sim12.shape", mat_sim12.shape
        dims_sim12 = dims_sim1[:]
        dims_sim12[1] = dims_sim1[1] + dims_sim2[1]
        
        d_param = {'at': range(2,3,1),
           'sr': mdp.numx.arange(1,2,0.2),
           'tau': range(1,10,1),
           'is': mdp.numx.arange(0.5,2,0.5)}
        dims = extract_dims_from_param_dic(param, d_param)
        print "dims",dims
        list_sim = Parse_file.io_std_files.load_file(
                                          full_file_path="/home/xavier/Work/Prog/Oger/workspace/XavReservoir/RES_TEMP/cv_timit_fabian_1500___all_mean_error_sim3",
                                          option='rb')
        mat_sim3 = extract_mat_from_list(list_sim, dimensions=dims)
        print "mat_sim3.shape", mat_sim3.shape
        dims_sim3 = dims
        
        d_param = {'at': range(2,3,1),
           'sr': mdp.numx.arange(0.1,1,0.2),
           'tau': range(1,10,1),
           'is': mdp.numx.arange(0.5,2,0.5)}
        dims = extract_dims_from_param_dic(param, d_param)
        print "dims",dims
        list_sim = Parse_file.io_std_files.load_file(
                                          full_file_path="/home/xavier/Work/Prog/Oger/workspace/XavReservoir/RES_TEMP/cv_timit_fabian_1500___all_mean_error_sim4",
                                          option='rb')
        mat_sim4 = extract_mat_from_list(list_sim, dimensions=dims)
        print "mat_sim4.shape", mat_sim4.shape
        dims_sim4 = dims
        
        mat_sim43 = mdp.numx.concatenate((mat_sim4, mat_sim3), axis=1)
        print "mat_sim43.shape", mat_sim43.shape
        dims_sim43 = dims_sim4[:]
        dims_sim43[1] = dims_sim4[1] + dims_sim3[1]
        
        mat_sim1243 = mdp.numx.concatenate((mat_sim12, mat_sim43), axis=0)
        print "mat_sim1243.shape", mat_sim1243.shape
        dims_sim1243 = dims_sim12[:]
        dims_sim1243[1] = dims_sim12[0] + dims_sim43[0]
        
        
        d_param = {'at': range(1,3,1),
           'sr': mdp.numx.concatenate((mdp.numx.arange(0.1,1,0.2),mdp.numx.arange(1,2,0.2)), axis=1),
           'tau': range(1,10,1),
           'is': mdp.numx.arange(0.5,2,0.5)}
        nr_sim = 1243
        plot_err_matrix(mat=mat_sim1243, param=param, param_values=d_param, title="_mean_errors_sim"+str(nr_sim)+"fusion",
                        filename_save_fig="../../RES_TEMP/cv-explor_timit_fabian_1500")
        
        

    def plot_sim_2011_11_22soir():
        import Parse_file.io_std_files
        param = ['at', 'sr', 'tau', 'is']
        
#        ## For sim 1:
#        nr_sim = 1
#        d_param = {'at': range(1,2,1),
#                   'sr': mdp.numx.arange(0.1,1,0.2),
#                   'tau': range(1,10,1),
#                   'is': mdp.numx.arange(0.5,2,0.5)}
#        ## For sim 2:
#        nr_sim = 2
#        d_param = {'at': range(1,2,1),
#                   'sr': mdp.numx.arange(1,2,0.2),
#                   'tau': range(1,10,1),
#                   'is': mdp.numx.arange(0.5,2,0.5)}
#        ## For sim 3:
#        nr_sim = 3
#        d_param = {'at': range(2,3,1),
#                   'sr': mdp.numx.arange(1,2,0.2),
#                   'tau': range(1,10,1),
#                   'is': mdp.numx.arange(0.5,2,0.5)}
        ## For sim 4:
        nr_sim = 4
        d_param = {'at': range(2,3,1),
                   'sr': mdp.numx.arange(0.1,1,0.2),
                   'tau': range(1,10,1),
                   'is': mdp.numx.arange(0.5,2,0.5)}
        
        dims = extract_dims_from_param_dic(param, d_param)
        (dims, param, param_ignored) = check_for_unity_dim(dims, param)
        ## if there is some parameters ignored, add there single value to the subtitle
        subtitle_param = ""
        if param_ignored!=[]:
            for p in param_ignored:
                subtitle_param = subtitle_param+"_"+str(p)+str(d_param[p][0])
        #extract all the possible combinaisons of plots for mean and standard deviation of errors
        for elt in ['mean', 'std']:
            list_sim = Parse_file.io_std_files.load_file(
                                              full_file_path="/home/xavier/Work/Prog/Oger/workspace/XavReservoir/RES_TEMP/cv_timit_fabian_1500___all_"+elt+"_error_sim"+str(nr_sim),
                                              option='rb')
            mat_sim = extract_mat_from_list(list_sim, dimensions=dims)
            plot_err_matrix(mat=mat_sim, param=param, param_values=d_param, title="_"+elt+"_errors_sim"+str(nr_sim)+subtitle_param,
                            filename_save_fig="../../RES_TEMP/cv-explor_timit_fabian_1500")
        
#    example_err_plot_from_list_err()
#    plot_sim_2011_11_22soir()
    plot_sim_2011_11_22soir_fusion_sim()
    
    