#!/usr/bin/env python


def filter_list(list, *conditions):
    condition_fail = False
    filtered_list = []

    for e in list:
        for condition in conditions:
            if not condition(e):
                condition_fail = True
                break
        
        if not condition_fail:
            filtered_list.append(e)

    return filtered_list
