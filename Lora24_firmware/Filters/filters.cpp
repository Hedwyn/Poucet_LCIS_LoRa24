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
 * @file filters.cpp
 * @author Baptiste Pestourie
 * @date 2021 March 26th
 * @brief Lightweight and efficient implementation of moving median filters for embedded applications - include FiFo, Linked Lists and Moving Median implementations
 */

#include "filters.h"

void LinkedElement::remove()
{
    if (prevElement != NULL)
    {
        prevElement->nextElement = nextElement;
    }
    if (nextElement != NULL)
    {
        nextElement->prevElement = prevElement;
    }
#ifdef ENABLE_DYNAMIC_ALLOCATION
    delete(this); 
#endif
}

LinkedList::LinkedList()
{
    head = NULL;
    tail = NULL;
}

list_retcode LinkedList::remove(int index)
{
    list_retcode ret= OPE_SUCCESS;
    int i = 0;
    LinkedElement *ite = head;
    while ((ite != NULL) && (i < index))
    {
        ite = ite->nextElement;
        i++;
    }
    if (ite != NULL)
    {
        ite->remove();
    }
    else {
        ret = LIST_TOO_SHORT;
    }
    return(ret);
}

list_retcode LinkedList::remove(LinkedElement *element)
{
    list_retcode ret = OPE_SUCCESS;
    if (element != NULL)
    {
        if (element == head)
        {
            head = element->nextElement;
        }
        if (element == tail)
        {
            tail = element->prevElement;
        }
        // delete(element);
        element->remove();
    }
    else 
    {
        ret = NULL_PTR;
    }
    return(ret);
}

void LinkedList::insert(LinkedElement *element)
{
    element->prevElement = NULL;
    element->nextElement = head;
    if (head != NULL)
    {
        head->prevElement = element;
    }
    else 
    {
        tail = element;
    }
    head = element;
}

list_retcode LinkedList::insert(LinkedElement *element, int index)
{
    list_retcode ret = OPE_SUCCESS;
    if (index == 0)
    {
        /* insert() already inserts at index 0 (as new head) by default */
        insert(element);
    }
    else 
    {
        int i = 1;
        LinkedElement *ite = head;
        while ((ite != NULL) && (i < index))
        {
            ite = ite->nextElement;
            i++;
        }
        if (ite == NULL)
        {
            if (i == index) 
            {
                append(element);
            }
            else {
                ret = LIST_TOO_SHORT;
            }
        }
        else {
            element->prevElement = ite;
            element->nextElement = ite->nextElement;
            ite->nextElement = element;
        }
    } 
    return(ret);
}

void LinkedList::append(LinkedElement *element)
{
    if (tail == NULL)
    {
        insert(element);
    }
    else {
        tail->nextElement = element;
        element->prevElement = tail;
        element->nextElement = NULL;
        tail = element;
    }

}

void LinkedList::insertSorted(LinkedElement *element)
{
    if ( (head == NULL) || ( element->value <= head-> value) )
    {
        insert(element);
    }
    else {
        LinkedElement *ite = head;
        while ( (ite->nextElement != NULL) && (element->value > ite->nextElement->value))
        {
            ite = ite->nextElement;
        }
        if (ite->nextElement == NULL)
        {
            append(element);
        }
        else 
        {
            element->nextElement = ite->nextElement;
            element->prevElement = ite;
            ite->nextElement->prevElement = element;
            ite->nextElement = element;
        }
        
    }
}

double* LinkedList::get(int index, double *value)
{
    double *ret= NULL;
    LinkedElement *ite = head;
    int i = 0;
    while ( (ite != NULL) && (i < index))
    {
        ite = ite->nextElement;
        i++;
    }
    if (ite != NULL)
    {
        *value = ite->value;
        ret = value;
    }

    return(ret);
}

int LinkedList::getDepth()
{
    LinkedElement *ite = head;
    int depth = 0;
    while (ite != NULL)
    {
        ite = ite->nextElement;
        depth++;
    }
    return depth;
}

template<size_t N> Fifo<N>::Fifo(const int length):length(length)
{
    /* allocating an array of size length */
#ifdef ENABLE_DYNAMIC_ALLOCATION
    elements = new LinkedElement*[length];
#endif
    inIndex =  0;
    outIndex = 0;
    isFull = false;
}

template<size_t N> LinkedElement* Fifo<N>::stack(LinkedElement* element)
{
    LinkedElement *ret = NULL;
    if (isFull)
    {
        ret = unstack();
    }
    elements[inIndex] = element;
    inIndex = (inIndex + 1) % length;
    isFull = (inIndex == outIndex);
    return(ret);
}

template<size_t N> int Fifo<N>::getHeight()
{
    return(isFull?length:( (inIndex - outIndex) % length));
}

template<size_t N> LinkedElement* Fifo<N>::unstack()
{
    LinkedElement *ret;
    if (!isFull && (outIndex == inIndex))
    {
        /* stack is empty */
        ret = NULL;
    } 
    else {
        ret = elements[outIndex];
        outIndex = (outIndex + 1) % length;
    }
    isFull = false;      
    return(ret);
}

template<size_t N> double MovingMedian<N>::compute()
{
   double res = 0;
   if (this->getHeight() % 2 == 1)
   {
       get((this->getHeight() + 1) / 2, &res);
   }
   else 
   {
       res = compute(1);
   }
   return(res);
}

template<size_t N> double MovingMedian<N>::compute(int meanLength)
{
    double temp, sum = 0;
    int startIndex = (this->getHeight() - meanLength) / 2;
    int stopIndex = startIndex + meanLength;
    for (int i = startIndex; i < stopIndex; i++)
    {
        get(i, &temp);
        sum += temp;
    }
    return(sum / meanLength);
}

template<size_t N> LinkedElement* MovingMedian<N>::unstack()
{
    remove(Fifo<N>::unstack());
    return(NULL);
}

template<size_t N> void MovingMedian<N>::append(double value)
{
#ifndef ENABLE_DYNAMIC_ALLOCATION
    LinkedElement *e = &_elements[_nextAvailable];
    this->stack(e);
    e->value = value;
    insertSorted(e);
    _nextAvailable = (_nextAvailable + 1) % N;


#else
    LinkedElement *e = new LinkedElement(value);
    stack(e);
    insertSorted(e);
#endif

}

#ifdef RUN_TESTS_FILTERS
int main()
{
    /** Test parameters */
    int max_ite = 15;

    /* Test FIFO */
    Fifo<5> f;
    printf("\n*** Test FIFO\r\n");
    printf("Fifo length:%d\n", f.length);
    int i = 0;
    while (i++ < 7)
    {
        f.stack(new LinkedElement((double) i));
    }
    LinkedElement *val;
    i = 0;
    while (i++ < 9)
    {
        val = f.unstack();
        if (val != NULL)
        {
            printf("obtained %f\r\n", val->value);
        }
    }

    /* Test linked list */
    printf("\n*** Test Linked List\n");
    LinkedList l = LinkedList();
    LinkedList k = LinkedList();
    MovingMedian<8> m;// = MovingMedian<6>();
    LinkedElement *e, *j;
    i = 0;
    while (i < max_ite)
    {
        l.insert(new LinkedElement((double) i), 0);
        k.insertSorted(new LinkedElement( (double) (i * (max_ite - i) ) ) );
        i++;
    }
    double res;
    l.get(5, &res); 
    printf("\n*** Testing get: %f\n", res);
    l.remove(0);
    l.remove(3);

    e = l.head;
    j = k.head;
    printf("\n*** Checking LL unsorted values\n");
    while (e != NULL)
    {
        printf("e = %f\n", e->value);
        e = e->nextElement;
    }
    printf("\n*** Checking LL sorted values\n");

    while (j != NULL)
    {
        printf("j = %f\n", j->value);
        j = j->nextElement;
    }
    i = 0;
    printf("\n*** Checking MM values\n");
    i = 0;
    while (i < max_ite)
    {
        int sum = (i >4)?1:0;
        double value = i * (max_ite - i) + sum;
        printf("Appending %f\n", value);
        m.append(value);
        i++;
    }
    LinkedElement *ite = m.head;
    i = 0;
    while (ite != NULL)
    {
        printf("k =  %f\n", ite->value);
        ite = ite->nextElement;
        i++;
    }
    printf("MM real depth = %d\n", i);
    printf("Moving median result: %f\n", m.compute());
    printf("Moving median + average result: %f\n", m.compute(10));

    return 0;
}
#endif