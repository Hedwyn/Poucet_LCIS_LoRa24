#include "filters.h"

LinkedElement::~LinkedElement()
{
    if (prevElement != NULL)
    {
        prevElement->nextElement = nextElement;
    }
    if (nextElement != NULL)
    {
        nextElement->prevElement = prevElement;
    }
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
        delete ite;
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
        delete(element);
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
    // if (depth == 0)
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

Fifo::Fifo(const int length):length(length)
{
    /* allocating an array of size length */
    elements = new LinkedElement*[length];
    inIndex =  0;
    outIndex = 0;
    isFull = false;
}

void Fifo::stack(LinkedElement* element)
{
    if (isFull)
    {
        /* destroying reference to the elements unstacked to avoid dead pointers */
        unstack();
    }
    elements[inIndex] = element;
    inIndex = (inIndex + 1) % length;
    isFull = (inIndex == outIndex);
}

int Fifo::getHeight()
{
    return(isFull?length:( (inIndex - outIndex) % length));
}

LinkedElement* Fifo::unstack()
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

double MovingMedian::compute()
{
   double res = 0;
   if (getHeight() % 2 == 1)
   {
       get((getHeight() + 1) / 2, &res);
   }
   else 
   {
       res = compute(1);
   }
   return(res);
}

double MovingMedian::compute(int meanLength)
{
    double temp, sum = 0;
    int startIndex = (getHeight() - meanLength) / 2;
    int stopIndex = startIndex + meanLength;
    for (int i = startIndex; i < stopIndex; i++)
    {
        get(i, &temp);
        sum += temp;
    }
    return(sum / meanLength);
}

LinkedElement* MovingMedian::unstack()
{
    remove(Fifo::unstack());
    return(NULL);
}

void MovingMedian::append(double value)
{
    LinkedElement *e = new LinkedElement(value);
    stack(e);
    insertSorted(e);
}

// int main()
// {
//     /* Test FIFO */
//     Fifo f = Fifo(5);
//     printf("\n*** Test FIFO\r\n");
//     int i = 0;
//     while (i++ < 7)
//     {
//         f.stack(new LinkedElement((double) i));
//     }
//     LinkedElement *val;
//     i = 0;
//     while (i++ < 9)
//     {
//         val = f.unstack();
//         if (val != NULL)
//         {
//             printf("obtained %f\r\n", val->value);
//         }
//     }

//     /* Test linked list */
//     printf("\n*** Test Linked List\n");
//     LinkedList l = LinkedList();
//     LinkedList k = LinkedList();
//     MovingMedian m = MovingMedian(3);
//     LinkedElement *e, *j;
//     i = 0;
//     int max_ite = 10;
//     while (i < max_ite)
//     {
//         l.insert(new LinkedElement((double) i), 0);
//         k.insertSorted(new LinkedElement( (double) (i * (max_ite - i) ) ) );
//         i++;
//     }
//     double res;
//     l.get(5, &res); 
//     printf("\n*** Testing get: %f\n", res);
//     l.remove(0);
//     l.remove(3);

//     e = l.head;
//     j = k.head;
//     printf("\n*** Checking LL unsorted values\n");
//     while (e != NULL)
//     {
//         printf("e = %f\n", e->value);
//         e = e->nextElement;
//     }
//     printf("\n*** Checking LL sorted values\n");

//     while (j != NULL)
//     {
//         printf("j = %f\n", j->value);
//         j = j->nextElement;
//     }
//     i = 0;
//     printf("\n*** Checking MM values\n");
//     i = 0;
//     while (i < max_ite)
//     {
//         int sum = (i >4)?1:0;
//         double value = i * (max_ite - i) + sum;
//         printf("Appending %f\n", value);
//         m.append(value);
//         i++;
//     }
//     LinkedElement *ite = m.head;
//     i = 0;
//     while (ite != NULL)
//     {
//         printf("k =  %f\n", ite->value);
//         ite = ite->nextElement;
//         i++;
//     }
//     printf("MM real depth = %d\n", i);
//     printf("Moving median result: %f\n", m.compute());
//     printf("Moving median + average result: %f\n", m.compute(10));

//     return 0;
// }