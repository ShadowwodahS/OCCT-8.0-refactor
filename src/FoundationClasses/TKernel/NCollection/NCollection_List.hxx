#ifndef NCollection_List_HeaderFile
#define NCollection_List_HeaderFile

#include <NCollection_OccAllocator.hxx>
#include <Standard_NoSuchObject.hxx>
#include <Standard_Assert.hxx>
#include <Standard_Transient.hxx>
#include <NCollection_DefaultHasher.hxx>
#include <NCollection_StlIterator.hxx>
#include <list>

/**
 * Purpose:      Simple list to link  items together keeping the first
 *               and the last one.
 *               Now implemented as a wrapper around std::list with OCCT allocator.
 */
template <class TheItemType>
class NCollection_List : public std::list<TheItemType, NCollection_OccAllocator<TheItemType>>
{
public:
  //! STL-compliant typedef for value type
  typedef TheItemType value_type;
  typedef std::list<TheItemType, NCollection_OccAllocator<TheItemType>> BaseList;

  public:
    //! Compatibility Iterator class (Universal Iterator)
    class Iterator_ : public BaseList::iterator
    {
    public:
      //! Default constructor
      Iterator_() : BaseList::iterator(), myEnd() {}
      
      //! Constructor from list
      Iterator_(const NCollection_List& theList) 
        : BaseList::iterator(const_cast<NCollection_List&>(theList).BaseList::begin()),
          myEnd(const_cast<NCollection_List&>(theList).BaseList::end()) {}
          
      //! Constructor from iterators
      Iterator_(const typename BaseList::iterator& it, const typename BaseList::iterator& end) 
        : BaseList::iterator(it), myEnd(end) {}

      void Init(const NCollection_List& theList)
      {
        BaseList::iterator::operator=(const_cast<NCollection_List&>(theList).BaseList::begin());
        myEnd = const_cast<NCollection_List&>(theList).BaseList::end();
      }

      void Initialize(const NCollection_List& theList) { Init(theList); }

      Standard_Boolean More() const { return *this != myEnd; }
      void Next() { (*this)++; }
      void Previous() { (*this)--; }
      const TheItemType& Value() const { return **this; }
      
      //! Compatibility: ChangeValue performs const_cast, matching OCCT's original behavior
      TheItemType& ChangeValue() const { return const_cast<TheItemType&>(**this); }

      //! Compatibility with legacy code calling .Iterator() on the result of begin()
      Iterator_& Iterator() { return *this; }
      const Iterator_& Iterator() const { return *this; }

      //! Performs comparison of two iterators.
      Standard_Boolean IsEqual(const Iterator_& theOther) const
      {
        return (const BaseList::iterator&)*this == (const BaseList::iterator&)theOther;
      }

    private:
      typename BaseList::iterator myEnd;
    };

    //! Both iterator types point to the same Universal Iterator for maximum compatibility
    typedef Iterator_ Iterator;
    typedef NCollection_StlIterator<std::bidirectional_iterator_tag, Iterator, TheItemType, false> iterator;
    typedef NCollection_StlIterator<std::bidirectional_iterator_tag, Iterator, TheItemType, true>  const_iterator;
    typedef iterator       const_Iterator; // legacy

public:
  // ---------- PUBLIC METHODS ------------

  //! Empty constructor.
  NCollection_List() : BaseList(NCollection_OccAllocator<TheItemType>()) {}

  //! Constructor
  explicit NCollection_List(const Handle(NCollection_BaseAllocator)& theAllocator)
    : BaseList(NCollection_OccAllocator<TheItemType>(theAllocator)) {}

  //! Returns attached allocator
  const Handle(NCollection_BaseAllocator)& Allocator() const 
  { 
    return this->get_allocator().Allocator(); 
  }

  //! Copy constructor
  NCollection_List(const NCollection_List& theOther) : BaseList(theOther) {}

  //! Move constructor
  NCollection_List(NCollection_List&& theOther) noexcept : BaseList(std::move(theOther)) {}

  //! Begin / End overrides to return compatibility wrappers
  iterator  begin() { return Iterator(BaseList::begin(), BaseList::end()); }
  iterator  end()   { return Iterator(BaseList::end(), BaseList::end()); }
  const_iterator  begin() const { return Iterator(const_cast<NCollection_List&>(*this).BaseList::begin(), const_cast<NCollection_List&>(*this).BaseList::end()); }
  const_iterator  end()   const { return Iterator(const_cast<NCollection_List&>(*this).BaseList::end(), const_cast<NCollection_List&>(*this).BaseList::end()); }
  const_iterator  cbegin() const { return begin(); }
  const_iterator  cend()   const { return end(); }

  //! Size - Number of items
  Standard_Integer Size() const { return static_cast<Standard_Integer>(this->size()); }
  Standard_Integer Extent() const { return Size(); }

  //! IsEmpty - Query if the list is empty
  Standard_Boolean IsEmpty() const { return this->empty(); }

  //! Replace this list by the items of another list (theOther parameter).
  NCollection_List& Assign(const NCollection_List& theOther)
  {
    if (this != &theOther) {
      BaseList::operator=(theOther);
    }
    return *this;
  }

  //! Replacement operator
  NCollection_List& operator=(const NCollection_List& theOther) { return Assign(theOther); }

  //! Move operator
  NCollection_List& operator=(NCollection_List&& theOther) noexcept
  {
    BaseList::operator=(std::move(theOther));
    return *this;
  }

  //! Clear this list
  void Clear(const Handle(NCollection_BaseAllocator)& theAllocator = 0L)
  {
    this->clear();
    if (!theAllocator.IsNull()) {
      this->get_allocator().SetAllocator(theAllocator);
    }
  }

  //! First item
  const TheItemType& First() const
  {
    Standard_NoSuchObject_Raise_if(this->empty(), "NCollection_List::First");
    return this->front();
  }

  //! First item (non-const)
  TheItemType& First()
  {
    Standard_NoSuchObject_Raise_if(this->empty(), "NCollection_List::First");
    return this->front();
  }

  //! Last item
  const TheItemType& Last() const
  {
    Standard_NoSuchObject_Raise_if(this->empty(), "NCollection_List::Last");
    return this->back();
  }

  //! Last item (non-const)
  TheItemType& Last()
  {
    Standard_NoSuchObject_Raise_if(this->empty(), "NCollection_List::Last");
    return this->back();
  }

  //! Append one item at the end
  TheItemType& Append(const TheItemType& theItem)
  {
    this->push_back(theItem);
    return this->back();
  }

  //! Append one item at the end
  TheItemType& Append(TheItemType&& theItem)
  {
    this->push_back(std::forward<TheItemType>(theItem));
    return this->back();
  }

  //! Append one item at the end and return iterator to it
  TheItemType& Append(const TheItemType& theItem, Iterator& theIter)
  {
    this->push_back(theItem);
    theIter = Iterator(--this->BaseList::end(), this->BaseList::end());
    return this->back();
  }

  //! Append one item at the end and return iterator to it
  TheItemType& Append(TheItemType&& theItem, Iterator& theIter)
  {
    this->push_back(std::forward<TheItemType>(theItem));
    theIter = Iterator(--this->BaseList::end(), this->BaseList::end());
    return this->back();
  }

  //! Append another list at the end.
  //! After this operation, theOther list will be cleared.
  void Append(NCollection_List& theOther)
  {
    if (this == &theOther || theOther.empty()) return;
    if (this->get_allocator().Allocator() == theOther.get_allocator().Allocator())
    {
      this->splice(this->BaseList::end(), theOther);
    }
    else
    {
      this->insert(this->BaseList::end(), theOther.begin(), theOther.end());
      theOther.clear();
    }
  }

  //! Prepend one item at the beginning
  TheItemType& Prepend(const TheItemType& theItem)
  {
    this->push_front(theItem);
    return this->front();
  }

  //! Prepend one item at the beginning
  TheItemType& Prepend(TheItemType&& theItem)
  {
    this->push_front(std::forward<TheItemType>(theItem));
    return this->front();
  }

  //! Prepend one item at the beginning and return iterator to it
  TheItemType& Prepend(const TheItemType& theItem, Iterator& theIter)
  {
    this->push_front(theItem);
    theIter = Iterator(this->BaseList::begin(), this->BaseList::end());
    return this->front();
  }

  //! Prepend one item at the beginning and return iterator to it
  TheItemType& Prepend(TheItemType&& theItem, Iterator& theIter)
  {
    this->push_front(std::forward<TheItemType>(theItem));
    theIter = Iterator(this->BaseList::begin(), this->BaseList::end());
    return this->front();
  }

  //! Prepend another list at the beginning
  void Prepend(NCollection_List& theOther)
  {
    if (this == &theOther || theOther.empty()) return;
    if (this->get_allocator().Allocator() == theOther.get_allocator().Allocator())
    {
      this->splice(this->BaseList::begin(), theOther);
    }
    else
    {
      this->insert(this->BaseList::begin(), theOther.begin(), theOther.end());
      theOther.clear();
    }
  }

  //! RemoveFirst item
  void RemoveFirst()
  {
    if (!this->empty()) this->pop_front();
  }

  //! Remove item pointed by iterator theIter;
  //! theIter is then set to the next item
  void Remove(iterator& theIter) { 
    theIter = Iterator(this->erase(theIter.Iterator()), this->BaseList::end()); 
  }

  //! Remove item pointed by legacy iterator theIter;
  //! theIter is then set to the next item
  void Remove(Iterator& theIter) { 
    theIter = Iterator(this->erase(theIter), this->BaseList::end()); 
  }

  //! Remove the first occurrence of the object.
  template <typename TheValueType>
  Standard_Boolean Remove(const TheValueType& theObject)
  {
    for (auto it = this->begin(); it != this->end(); ++it) {
      if (IsEqual(*it, theObject)) {
        this->erase(it.Iterator());
        return Standard_True;
      }
    }
    return Standard_False;
  }

  //! InsertBefore
  TheItemType& InsertBefore(const TheItemType& theItem, iterator& theIter)
  {
    return *this->insert(theIter.Iterator(), theItem);
  }

  //! InsertBefore legacy
  TheItemType& InsertBefore(const TheItemType& theItem, Iterator& theIter)
  {
    return *this->insert(theIter, theItem);
  }

  //! InsertBefore
  TheItemType& InsertBefore(TheItemType&& theItem, iterator& theIter)
  {
    return *this->insert(theIter.Iterator(), std::forward<TheItemType>(theItem));
  }

  //! InsertBefore legacy
  TheItemType& InsertBefore(TheItemType&& theItem, Iterator& theIter)
  {
    return *this->insert(theIter, std::forward<TheItemType>(theItem));
  }

  //! InsertBefore another list
  void InsertBefore(NCollection_List& theOther, iterator& theIter)
  {
    if (this == &theOther || theOther.empty()) return;
    if (this->get_allocator().Allocator() == theOther.get_allocator().Allocator())
    {
      this->splice(theIter.Iterator(), theOther);
    }
    else
    {
      this->insert(theIter.Iterator(), theOther.begin(), theOther.end());
      theOther.clear();
    }
  }

  //! InsertBefore another list legacy
  void InsertBefore(NCollection_List& theOther, Iterator& theIter)
  {
    if (this == &theOther || theOther.empty()) return;
    if (this->get_allocator().Allocator() == theOther.get_allocator().Allocator())
    {
      this->splice(theIter, theOther);
    }
    else
    {
      this->insert(theIter, theOther.begin(), theOther.end());
      theOther.clear();
    }
  }

  //! InsertAfter
  TheItemType& InsertAfter(const TheItemType& theItem, iterator& theIter)
  {
    auto nextIt = theIter.Iterator();
    if (nextIt != this->BaseList::end()) ++nextIt;
    return *this->insert(nextIt, theItem);
  }

  //! InsertAfter legacy
  TheItemType& InsertAfter(const TheItemType& theItem, Iterator& theIter)
  {
    auto nextIt = theIter;
    if (nextIt != this->BaseList::end()) ++nextIt;
    return *this->insert(nextIt, theItem);
  }

  //! InsertAfter another list
  void InsertAfter(NCollection_List& theOther, iterator& theIter)
  {
    if (this == &theOther || theOther.empty()) return;
    auto nextIt = theIter.Iterator();
    if (nextIt != this->BaseList::end()) ++nextIt;
    if (this->get_allocator().Allocator() == theOther.get_allocator().Allocator())
    {
      this->splice(nextIt, theOther);
    }
    else
    {
      this->insert(nextIt, theOther.begin(), theOther.end());
      theOther.clear();
    }
  }

  //! InsertAfter another list legacy
  void InsertAfter(NCollection_List& theOther, Iterator& theIter)
  {
    if (this == &theOther || theOther.empty()) return;
    auto nextIt = theIter;
    if (nextIt != this->BaseList::end()) ++nextIt;
    if (this->get_allocator().Allocator() == theOther.get_allocator().Allocator())
    {
      this->splice(nextIt, theOther);
    }
    else
    {
      this->insert(nextIt, theOther.begin(), theOther.end());
      theOther.clear();
    }
  }

  //! Reverse the list
  void Reverse() { this->reverse(); }

  //! Return true if object is stored in the list.
  template <typename TheValueType>
  Standard_Boolean Contains(const TheValueType& theObject) const
  {
    for (const auto& it : *this) {
      if (IsEqual(it, theObject)) return Standard_True;
    }
    return Standard_False;
  }

  //! Destructor - clears the List
  virtual ~NCollection_List() { this->clear(); }

private:
  //! Helper to use OCCT's equality logic or fall back to operator==
  template <typename T1, typename T2>
  static Standard_Boolean IsEqual(const T1& theFirst, const T2& theSecond)
  {
    return (theFirst == theSecond);
  }
};

#endif
