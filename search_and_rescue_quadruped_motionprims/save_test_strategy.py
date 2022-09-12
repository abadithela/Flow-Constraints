# Computing and saving the cuts on the virtual game graph
from find_constraints import find_cuts
import _pickle as pickle


def find_cuts_and_save():
    G, node_dict, inv_node_dict, init, cuts, snr_to_nr, snr_to_label, label_to_snr = find_cuts()

    opt_dict = {'G': G, 'node_dict': node_dict, 'inv_node_dict':inv_node_dict, 'init': init, 'cuts': cuts, \
    'snr_to_nr':snr_to_nr, 'snr_to_label': snr_to_label, 'label_to_snr':label_to_snr}

    with open('stored_optimization_result.p', 'wb') as pckl_file:
        pickle.dump(opt_dict, pckl_file)

    return G, node_dict, inv_node_dict, init, cuts, snr_to_nr, snr_to_label, label_to_snr
