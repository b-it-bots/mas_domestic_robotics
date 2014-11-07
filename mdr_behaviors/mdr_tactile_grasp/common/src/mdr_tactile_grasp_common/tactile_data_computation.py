#!/usr/bin/env python
"""
This module contains functions to process and compute different tactile
features such as thresholding, contact region extraction,

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

SQUARE_MM_CONVERSION = 1000000  # Conversion factor from mm^2 to m^2.
# 3.4 by 3.4 for the proximal phalanges, however for the distal
# phalanges there are slight variations (+.15/-0.06).
CELL_SIZE = 3.4                 # mm
PRESSURE_RANGE = 250000         # Pa
MAX_DISPLAYED_VALUE = 4095      # According to Weiss Robotics


def detect_contact(matrix_list, threshold_list):
    """
    Detects which phalanx have a contact and which do not, based on a threshold value.

    Args:
        matrix_list: Int[[]]
            The phalanges (matrices) that are enabled with a tactile sensing
            array and their contact values.
        threshold_list: Int[]
            The minimum value of required contact for each phalanx.

    Returns:
        contact_list: Int[]
            Phalanges that have a contact greater or equal to their threshold
            value have a value of '1', a value of '0' is assigned otherwise.

    """
    contact_list = []

    for i, matrix in enumerate(matrix_list):
        if max(matrix) < threshold_list[i]:
            contact_list.append(0)
        else:
            contact_list.append(1)

    return contact_list


def detect_threshold(matrix_list, threshold_list):
    """
    Detects which phalanx should stop moving, based on a threshold value.

    Args:
        matrix_list: Int[[]]
            The phalanges (matrices) that are enabled with a tactile sensing
            array and their contact values.
        threshold_list: Int[]
            The minimum value of required contact for each phalanx.
    Returns:
        stop_criteria: Int[]
            Phalanges that have a contact greater or equal to their threshold
            value have a value of '0', a value of '1' is assigned otherwise.

    """
    # ToDo: fix --> added first element (does not exist), since there are 7
    # velocities and only 6 fingers....
    stop_criteria = [0]

    for i, matrix in enumerate(matrix_list):
        if max(matrix) < threshold_list[i]:
            stop_criteria.append(1)
        else:
            stop_criteria.append(0)

    return stop_criteria


def threshold_contact_matrices(matrix_list, contact_threshold):
    """
    Sets to zero all the elements of each matrix in the matrix list that are below
    their respective threshold.

    Args:
        matrix_list: Int[[]]
            The phalanges (matrices) that are enabled with a tactile sensing
            array and their contact values.
        contact_threshold: Int[]
            The minimum value of required contact for each phalanx.
    Returns:
        stop_criteria: Int[[]]
            The filtered contact matrices. Cells that have a value below their
            threshold are set to zero and the others remain with their original
            contact values.

    """
    # reset variables
    thresholded_contact_matrices = [[] for _ in range(len(matrix_list))]

    for i, matrix in enumerate(matrix_list):
        for cell in matrix:
            if cell >= contact_threshold[i]:
                thresholded_contact_matrices[i].append(cell)
            else:
                thresholded_contact_matrices[i].append(0)

    return thresholded_contact_matrices


def label_regions(thresholded_matrices, array_widths):
    """
    Labels the regions where contact is located using the connected-component labeling
    algorithm with a 4-connectivity criteria.

    Args:
        thresholded_matrices: Int[[]]
            The thresholded contact matrices.
        array_widths: Int[]
            The number of columns that the each tactile matrix has.
    Returns:
        regions: Int[[]]
            The contact regions of each tactile matrix.

    """
    temporary_list = [[] for _ in range(len(thresholded_matrices))]
    regions = [[] for _ in range(len(thresholded_matrices))]

    for i, matrix in enumerate(thresholded_matrices):
        temporary_list[i] = [0] * len(matrix)

        # fist pass
        temporary_list[i], equivalences = generate_contact_regions(
            matrix, array_widths[i])

        # second pass
        regions[i] = merge_equivalent_regions(temporary_list[i], equivalences)

    return regions


def generate_contact_regions(thresholded_matrix, array_width):
    """
    This is the first pass of the 'connected-component labeling' algorithm.
    It returns a list containing the contact regions of a single filtered matrix.
    It uses four-connectivity.

   Args:
        thresholded_matrix: Int[[]]
            A thresholded contact matrix.
        array_widths: Int[]
            The number of columns that the each tactile matrix has.
    Returns:
        temporary_list, : Int[]
            The contact regions with their respect labels.
        equivalences: Int[]
            The regions that are marked with different label, but are the same.

    """
    # reset variables
    equivalences = {}
    set_id = 0
    label = 1

    temporary_list = [0] * len(thresholded_matrix)

    for i, cell in enumerate(thresholded_matrix):
        if cell == 0:
            pass
        else:
            # look for neighbors and assign them the lowest label
            left_cell, above_cell = \
                generate_neighbors_4_connect(i, array_width)

            # assign new label
            if (left_cell == -1 or thresholded_matrix[left_cell] == 0) and \
                    (above_cell == -1 or thresholded_matrix[above_cell] == 0):
                temporary_list[i] = label
                label += 1

            # assign north label
            elif left_cell == -1 or thresholded_matrix[left_cell] == 0:
                temporary_list[i] = temporary_list[above_cell]

            # assign west label
            elif above_cell == -1 or thresholded_matrix[above_cell] == 0:
                temporary_list[i] = temporary_list[left_cell]

            # when north and west label are the same
            elif temporary_list[above_cell] == temporary_list[left_cell]:
                temporary_list[i] = temporary_list[left_cell]

            # assign different label and equivalence relationship
            else:
                temporary_list[i] = \
                    min(temporary_list[left_cell],
                        temporary_list[above_cell])

                equivalences[set_id] = [temporary_list[left_cell],
                                        temporary_list[above_cell]]
                set_id += 1

    return temporary_list, equivalences


def merge_equivalent_regions(temporary_list, equivalences):
    """
    This is the second pass of the 'connected-component labeling' algorithm.
    It returns a list containing the regions with unique labels.

   Args:
        temporary_list: Int[[]]
            The contact regions with their respect labels.
        equivalences: Int[]
            The regions that are marked with different label, but are the same.
    Returns:
        regions: Int[]
            The unique contact regions.

    """
    count = 0

    regions = temporary_list

    for i in temporary_list:
        for set_id, _ in equivalences.items():
            if i in equivalences[set_id] and i > min(equivalences[set_id]):
                regions[count] = min(equivalences[set_id])
            else:
                pass
        count += 1

    return regions


def generate_neighbors_4_connect(current_cell, array_width):
    """
    Calculates the north (above) and west (to the left) neighbors of the current cell.
    If a neighbor does not exist it assigns it the value of '-1'.
    It uses 4-connectivity to look for neighbors.

   Args:
        current_cel: Int
            The number (index) of the current tactel.
        array_width: Int
            The number of columns the tactile matrix has.
    Returns:
        west_neighbor: Int
            The number (index) of the west neighbor of the current tactel.
        north_neighbor: Int
            The number (index) of the north neighbor of the current tactel.

    """
    x_coordinate = current_cell / array_width
    y_coordinate = current_cell % array_width

    if current_cell == 0:
        west_neighbor = -1
        north_neighbor = -1

    elif x_coordinate == 0:
        north_neighbor = -1
        west_neighbor = (array_width * x_coordinate) + (y_coordinate - 1)

    elif y_coordinate == 0:
        west_neighbor = -1
        north_neighbor = (array_width * (x_coordinate - 1)) + y_coordinate

    else:
        west_neighbor = (array_width * x_coordinate) + (y_coordinate - 1)
        north_neighbor = (array_width * (x_coordinate - 1)) + y_coordinate

    return west_neighbor, north_neighbor


def generate_neighbors_8_connect(current_cell, array_width):
    """
    Calculates the the west, north-west, the north, the north-east neighbors
    of the current cell. If a neighbor does not exist it assigns it the value
    of '-1'. It uses 8-connectivity to look for neighbors.

   Args:
        current_cel: Int
            The number (index) of the current tactel.
        array_width: Int
            The number of columns the tactile matrix has.
    Returns:
        west_neighbor: Int
            The number (index) of the west neighbor of the current tactel.
        north_west_neighbor: Int
           The number (index) of the north-west neighbor of the current tactel.
        north_neighbor: Int
            The number (index) of the north neighbor of the current tactel.
        north_east_neighbor: Int
            The number (index) of the north-east neighbor of the current tactel.

    """
    x_coordinate = current_cell / array_width
    y_coordinate = current_cell % array_width

    if current_cell == 0:
        west_neighbor = -1
        north_neighbor = -1
        north_west_neighbor = -1
        north_east_neighbor = -1

    elif x_coordinate == 0:
        north_neighbor = -1
        north_west_neighbor = -1
        north_east_neighbor = -1
        west_neighbor = (array_width * x_coordinate) + (y_coordinate - 1)

    elif y_coordinate == 0:
        west_neighbor = -1
        north_west_neighbor = -1
        north_neighbor = (array_width * (x_coordinate - 1)) + y_coordinate
        north_east_neighbor = (array_width * (x_coordinate - 1)) + \
                              (y_coordinate + 1)

    elif y_coordinate == array_width - 1:
        west_neighbor = (array_width * x_coordinate) + (y_coordinate - 1)
        north_neighbor = (array_width * (x_coordinate - 1)) + y_coordinate
        north_west_neighbor = (array_width * (x_coordinate - 1)) + \
                              (y_coordinate - 1)
        north_east_neighbor = -1

    else:
        west_neighbor = (array_width * x_coordinate) + (y_coordinate - 1)
        north_neighbor = (array_width * (x_coordinate - 1)) + y_coordinate
        north_west_neighbor = (array_width * (x_coordinate - 1)) + \
                              (y_coordinate - 1)
        north_east_neighbor = (array_width * (x_coordinate - 1)) + \
                              (y_coordinate + 1)

    return west_neighbor, north_west_neighbor, north_neighbor, \
        north_east_neighbor


def extract_contact_region(region_lists):
    """
    Extracts the largest contact region. based on [1].

    [1] Li, Qiang, Christof Elbrechter, Robert Haschke, and Helge Ritter.
    "Integrating vision, haptics and proprioception into a feedback controller
    for in-hand manipulation of unknown objects."

   Args:
        region_lists: Int[[]]
            The contact regions of each tactile matrix.
    Returns:
        largest_region: Int[[]]
            The largest contact region of each tactile matrix.

    """
    largest_regions = region_lists
    largest_labels = [[] for _ in range(len(region_lists))]

    # create a list of regions with the highest frequency
    for i, region in enumerate(region_lists):
        # reset variables
        frequency = {}

        if max(region) == 0:
            largest_labels[i] = 0
        else:
            for j in region:
                if j == 0:
                    pass
                else:
                    try:
                        if frequency[j]:
                            frequency[j] += 1
                    except KeyError:
                        frequency[j] = 1

            largest_labels[i] = max(frequency, key=frequency.get)

    # set every value that is not the largest region to zero
    for i, region in enumerate(region_lists):
        for j, cell in enumerate(region):
            if cell == largest_labels[i]:
                pass
            else:
                largest_regions[i][j] = 0

    return largest_regions


def calculate_contact_values(matrix_list, contact_regions):
    """
    Calculates the contact pressures values of the specified contact region for all
    phalanges. based on [1].

    [1] Li, Qiang, Christof Elbrechter, Robert Haschke, and Helge Ritter.
    "Integrating vision, haptics and proprioception into a feedback controller
    for in-hand manipulation of unknown objects."

    Args:
        matrix_list: Int[[]]
            The phalanges (matrices) that are enabled with a tactile sensing
            array and their contact values.
        contact_regions: Int[[]]
            The largest contact region of each tactile matrix.
    Returns:
        pressure_values: Int[[]]
            The pressure values of each tactel in the contact region of
            tactile matrix.

    """
    # reset variables
    pressure_values = [[] for _ in range(len(matrix_list))]

    for i, region in enumerate(contact_regions):
        for j, cell in enumerate(region):
            if cell != 0:
                pressure_values[i].append(matrix_list[i][j])
            else:
                pressure_values[i].append(0)

    return pressure_values


def locate_contact_position(forces_list, array_widths):
    """
    Locate the contact position (centroid) of each contact region for all phalanges.
    based on [1].

    [1] Li, Qiang, Christof Elbrechter, Robert Haschke, and Helge Ritter.
    "Integrating vision, haptics and proprioception into a feedback controller
    for in-hand manipulation of unknown objects."

    Args:
        forces_list: Float[[]]
            The contact forces of each cell in the contact region of each
            tactile matrix.
        array_widths: Int[]
            The number of columns that the each tactile matrix has.
    Returns:
        centroids: Int[[]]
            The location of the contact's centroid for each tactile matrix.

    """
    # reset variables
    centroids = [0] * len(forces_list)
    x_coordinates = [0] * len(forces_list)
    y_coordinates = [0] * len(forces_list)

    for i, phalange_forces in enumerate(forces_list):
        if sum(phalange_forces) == 0:
            centroids[i] = -1

        else:
            x_addition = 0
            y_addition = 0

            for j, cell_force in enumerate(phalange_forces):
                x_addition += cell_force * ((j / array_widths[i]) + 1)
                y_addition += cell_force * ((j % array_widths[i]) + 1)

            x_coordinates[i] = int(round(float(x_addition) /
                                   sum(phalange_forces)))
            y_coordinates[i] = int(round(float(y_addition) /
                                   sum(phalange_forces)))

            centroids[i] = ((x_coordinates[i] - 1) * array_widths[i]) + \
                           (y_coordinates[i] - 1)

    return centroids


def get_pressure_regions(pressures, regions):
    """
    Obtains the pressure values of each contact region as a dictionary, where the
    keys represent the labels of the contact regions and the values the pressures
    in each cell (tactel).
    It only calculates one phalanx (i.e. one tactile matrix).

    Args:
        pressures: Int[]
            The contact values (in scaled pressure units, i.e. 4095 is the
            maximum) in each cell for a single tactile matrix.
        regions: Int[[]]
            The labels of the contact regions for a single tactile matrix

    Returns:
        pressure_regions: Float{}
            The labels of the contact regions (keys) with their contact values
            for a single tactile matrix.

    """
    pressure_regions = {}

    for i, region in enumerate(regions):
        if region > 0:
            try:
                if pressure_regions[region]:
                    pressure_regions[region].append(pressures[i])
            except KeyError:
                pressure_regions[region] = [pressures[i]]

    return pressure_regions


def calculate_normal_force(contact_matrices):
    """
    Calculates the normal force (F) of every contact region, by multiplying their
    normalized pressure (P) times their area, it is performed for each contact matrix.
                F = P*A
    The normalized pressure (P), given in Pascals, of a contact region is calculated by
    multiplying the maximum pressure range (250KPa) by the sum of all the scaled pressure
    values (i.e. 4095 is the maximum value).
    The area (A), given in sq. mm, is calculated by multiplying the individual area of a
    cell times the number of cells that have a contact value.

    Args:
        contact_matrices: Int[[]]
            The contact regions of each phalanx (tactile matrix) with their
            scaled pressured values.

    Returns:
        normal_force: Float[]
            The normal force, in millinewtons (mN), of every contact region in
            each phalanx.

    """
    cell_area = CELL_SIZE * CELL_SIZE
    normal_forces = [[]] * len(contact_matrices)

    for i, matrix in enumerate(contact_matrices):
        normal_forces[i] = {}
        for label, pressures in matrix.items():
            number_of_contact_cells = len(pressures)
            sum_of_cell_values = sum(pressures)

            area = cell_area * number_of_contact_cells / SQUARE_MM_CONVERSION
            scaling_factor = float(MAX_DISPLAYED_VALUE) / PRESSURE_RANGE
            p_norm = sum_of_cell_values * scaling_factor
            normal_force = p_norm * area

            # Convert from newtons to millinewtons
            normal_forces[i][label] = normal_force * 1000

    return normal_forces


def get_strongest_regions(forces_list, regions_list):
    """
    Obtains the strongest region (i.e. the one that has the highest force) for
    each phalanx (tactile matrix).

    Args:
        forces_list: Float[]
            Contains the normal forces of every contact region of each phalanx.
        regions_list: Int[[]]
            Contains the labels of every contact region of each phalanx.

    Returns:
        strongest_regions: Float{}
            The strongest region is selected and the contact values in other
            regions are set to zero.

    """
    labels = []
    strongest_regions = regions_list

    for i, matrix in enumerate(forces_list):
        try:
            label = max(matrix, key=matrix.get)
            labels.append(label)
        except ValueError:
            labels.append(1)

    for i, region in enumerate(regions_list):
        for j, cell in enumerate(region):
            if cell != labels[i]:
                strongest_regions[i][j] = 0

    return strongest_regions


def get_maximum_pressures(matrix_list):
    """
    Obtains the value of the tactel with the highest pressure for each phalanx.

    Args:
        matrix_list: Int[[]]
            The phalanges (matrices) that are enabled with a tactile sensing
            array and their contact values.

    Returns:
        maximum_values: Int[]
            Maximum pressure values for each phalanx.

    """
    maximum_values = []

    for i, matrix in enumerate(matrix_list):
        maximum_values.append(max(matrix))

    return maximum_values


def get_average_pressures(matrix_list):
    """
    Obtains the value of the average pressure for each phalanx.

    Args:
        matrix_list: Int[[]]
            The phalanges (matrices) that are enabled with a tactile sensing
            array and their contact values.

    Returns:
        average_values: Int[]
            Average pressure values for each phalanx.

    """
    average_values = []

    for i, matrix in enumerate(matrix_list):
        average_value = sum(matrix) / float(len(matrix))
        average_values.append(round(average_value))

    return average_values
