/****************************************************************************
* Copyright (C) 2021 LCIS Laboratory - Baptiste Pestourie
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, in version 3.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* This program is part of the SecureLoc Project @https://github.com/Hedwyn/SecureLoc
 ****************************************************************************/

/**
 * \file filters.h
 * \author Baptiste Pestourie
 * \date 2021 March 26th
 * \brief Lightweight and efficient implementation of moving median filters for embedded applications - include FiFo, Linked Lists and Moving Median implementations
 */

#include <stdio.h>
#include <array>
#include <float.h>
#include <stdbool.h>

// #define ENABLE_DYNAMIC_ALLOCATION
#ifndef DEFAULT_LENGTH
    constexpr int DEFAULT_LENGTH = 10;
#endif
/** Error codes */

/**
 * \brief Different types of error codes returned during operations on Lists, Stacks or filters
 */ 
enum list_retcode
{
    OPE_SUCCESS = 1,
    LIST_TOO_SHORT,
    NULL_PTR
};

/**
 * \brief Chain link of a linked list. Contains two pointers, one to the previous, one to the next element.
 */ 
class LinkedElement
{
    public:
    LinkedElement(double value = 0, LinkedElement* nextElement = NULL, LinkedElement *prevElement = NULL):value(value), nextElement(nextElement), prevElement(prevElement) {}
    ~LinkedElement() {}
    void remove();
    double value;
    LinkedElement *nextElement; /**<Pointer to the next element in the chain*/
    LinkedElement *prevElement; /**<Pointer to the previous element in the chain*/
};

/**
 * \brief An implementation of a linked list. A linked list contains chained linked elements, which all contain a reference to their previous and next elements. Can be iterated forward or backward.
 */ 
class LinkedList
{
    public:
    LinkedList();
    int getDepth();
    LinkedElement *head;
    LinkedElement *tail;
    list_retcode remove(int index);
    list_retcode remove(LinkedElement *element);
    list_retcode insert(LinkedElement *element, int index);
    void insert(LinkedElement *element);
    virtual void append(LinkedElement *element);
    void insertSorted(LinkedElement *element);
    double *get(int index, double *value);
};

/**
 * \brief An implementation of a First-In First-Out (Fifo) Stack. When the stack is full, an element is automatically unstacked when stacking a new element.
 */ 
template<size_t N> class Fifo
{
    public:
    Fifo(const int length = N);
    const int length;
#ifndef ENABLE_DYNAMIC_ALLOCATION
    LinkedElement* elements[N];
#else
    LinkedElement** elements;
#endif
    LinkedElement* stack(LinkedElement* element); 
    virtual LinkedElement* unstack();
    int getHeight();

    private:
    int inIndex;
    int outIndex;
    bool isFull;
    
};

/**
 * \brief An implementation of a moving median filter, based on the combination of a Fifo and a linked list. A flexible definition of median is used; the median is based on a the mean of the N-centers elements rather than just one of two.
 */ 
template<size_t N> class MovingMedian: public LinkedList, public Fifo<N>
{
    public:
#ifdef ENABLE_DYNAMIC_ALLOCATION
    MovingMedian(const int length = N):LinkedList(), Fifo<N>(length) {}
#else
    MovingMedian():LinkedList(),Fifo<N>(), _nextAvailable(0) {}
#endif
    double compute();
    double compute(int meanLength);
    void append(double value);
    void accumulate(double value);
    LinkedElement* unstack();
    Fifo<N> *accumulator;

#ifndef ENABLE_DYNAMIC_ALLOCATION
    private:
    LinkedElement _elements[N];
    int _nextAvailable;
#endif
};