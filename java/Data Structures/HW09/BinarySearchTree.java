import java.io.*;

/** A class to represent a binary search tree.
 *  @author Koffman and Wolfgang
 */

public class BinarySearchTree <E extends Comparable <E>> extends BinaryTree <E> implements SearchTree <E> 
{
  /** Return value from the public add method. */
  protected boolean addReturn;
  /** Return value from the public delete method. */
  protected E deleteReturn;

  /** Starter method find.
      pre: The target object must implement
           the Comparable interface.
      @param target The Comparable object being sought
      @return The object, if found, otherwise null
   */
  public E find(E target) {
    return find(root, target);
  }

  /** Recursive find method.
      @param localRoot The local subtree’s root
      @param target The object being sought
      @return The object, if found, otherwise null
   */
  private E find(Node < E > localRoot, E target) {
    if (localRoot == null)
      return null;

    // Compare the target with the data field at the root.
    int compResult = target.compareTo(localRoot.data);
    if (compResult == 0)
      return localRoot.data;
    else if (compResult < 0)
      return find(localRoot.left, target);
    else
      return find(localRoot.right, target);
  }

  /** Starter method add.
     pre: The object to insert must implement the
          Comparable interface.
     @param item The object being inserted
     @return true if the object is inserted, false
             if the object already exists in the tree
   */
  	public boolean add(E item) {
  		// create a new node with item
  		Node<E> newNode = new Node<E>(item);
  		root = add(root, newNode);
  		return addReturn;
  	}

  /** Recursive add method.
      post: The data field addReturn is set true if the item is added to
            the tree, false if the item is already in the tree.
      @param localRoot The local root of the subtree
      @param newNode The node to be inserted
      @return The new local root that now contains the
              inserted item
   */
  	private Node <E> add(Node <E> localRoot, Node<E> newNode) {
  		// if tree is empty, insert newNode to root
  		if (localRoot == null) {
  			// item is not in the tree — insert it.
  			addReturn = true; // set addReturn by true
  			return newNode; // return localRoot
  		}
  		// if newNode.data equals localRoot.data, no insertion
  		// set addReturn by false, return localRoot
  		else if (newNode.data.compareTo(localRoot.data) == 0) {
  			// item is equal to localRoot.data
  			addReturn = false; // set addReturn by false
  			return localRoot; // return localRoot
  		}
  		// if newNode.data is smaller than localRoot.data, check root.left
  		// if root.left is null, insert newNode there and
  		// link nextOrder and previousOrder references between nodes
  		else if(newNode.data.compareTo(localRoot.data) < 0) {
  			// if localRoot.left is null, insert newNode there
  			if(localRoot.left == null) {
  				if(localRoot.previousOrder == null) {
  	  				localRoot.left = newNode; // insert newNode to root.left
  	  				newNode.nextOrder = localRoot; // newNode.nextOrder points localRoot
  	  				localRoot.previousOrder = newNode; // localRoot.previousOrder points newNode
  				}
  				else {
  					localRoot.left = newNode; // insert newNode to root.left
  					newNode.nextOrder = localRoot;
  					newNode.previousOrder = localRoot.previousOrder;
  					localRoot.previousOrder.nextOrder = newNode;
  					localRoot.previousOrder = newNode;
  				}
	  			
  				addReturn = true; // set addReturn by true
  	  			return localRoot; // return localRoot
  			}
  			// item is less than localRoot.data
  			localRoot.left = add(localRoot.left, newNode);
  			return localRoot; // return localRoot
  		}
  		// if newNode.data is bigger than localRoot.data, check right
  		// if root.right is null, insert newNode there and
  		// link nextOrder and previousOrder references between nodes
  		else {
  			// if localRoot.right is null, insert newNode there
  			if(localRoot.right == null) {
  				if(localRoot.nextOrder == null ) {
  					localRoot.right = newNode; // insert newNode to root.right
  					newNode.nextOrder = localRoot.nextOrder; // newNode.nextOrder points where localRoot.nextOrder points
  					newNode.previousOrder = localRoot; // newNode.previousOrder points localRoot
  					localRoot.nextOrder = newNode; // localRoot.nextOrder points newNode
  				}
  				else {
  					localRoot.right = newNode; // insert newNode to root.right
  					newNode.nextOrder = localRoot.nextOrder; // newNode.nextOrder points where localRoot.nextOrder points
  					newNode.previousOrder = localRoot; // newNode.previousOrder points localRoot
  					localRoot.nextOrder.previousOrder = newNode;
  					localRoot.nextOrder = newNode; // localRoot.nextOrder points newNode
  				}

  				addReturn = true; // set addReturn by true
  				return localRoot; // return localRoot
  			}
  			
  			// item is greater than localRoot.data
  			localRoot.right = add(localRoot.right, newNode);
  			return localRoot; // return localRoot
  		}
  	}

  /** Starter method delete.
      post: The object is not in the tree.
      @param target The object to be deleted
      @return The object deleted from the tree
              or null if the object was not in the tree
      @throws ClassCastException if target does not implement
              Comparable
   */
  public E delete(E target) {
	Node<E> deleteNode = new Node<E>(target);
    root = delete(root, deleteNode);
    return deleteReturn;
  }

  /** Recursive delete method.
      post: The item is not in the tree;
            deleteReturn is equal to the deleted item
            as it was stored in the tree or null
            if the item was not found.
      @param localRoot The root of the current subtree
      @param deleteNode The node to be deleted
      @return The modified local root that does not contain
              the item
   */
  private Node < E > delete(Node < E > localRoot, Node<E> deleteNode) {
    if (localRoot == null) {
    	// item is not in the tree.
    	deleteReturn = null;
      
    	return localRoot;
    }

    // Search for item to delete.
    int compResult = deleteNode.data.compareTo(localRoot.data);
    
    if(compResult < 0) {
    	// item is smaller than localRoot.data.
    	localRoot.left = delete(localRoot.left, deleteNode);
    	
    	return localRoot;
    }
    else if(compResult > 0) {
    	// item is larger than localRoot.data.
    	localRoot.right = delete(localRoot.right, deleteNode);
    	
    	return localRoot;
    }
    else {
      // item is at local root.
      deleteReturn = localRoot.data;
      
      if (localRoot.left == null) {
    	  // if localRoot is the rightmost leaf, its nextOrder points null
    	  if(localRoot.nextOrder == null) {
    		  // if there are not left and previousOrder node of localRoot, tree has just one node, root
    		  // so, localRoot.nextOrder can not point any node, it is set as null
    		  if(localRoot.previousOrder == null)
    			  localRoot.nextOrder = null;
    		  else
    			  localRoot.previousOrder.nextOrder = null;
    	  }
    	  // if localRoot.nextOrder doesn't point null, it's not the rightmost node
    	  else {
    		  // if localRoot.previousOrder is null, this node is the leftmost node 
    		  if(localRoot.previousOrder == null) {
    			  if(localRoot.right == null) {
    				  localRoot.nextOrder.previousOrder = null;
    				  localRoot.nextOrder = null;
    			  }
    			  else {
    				  localRoot.setData(localRoot.right.data); // set the leftmost node data by data in nextOrder
    				  localRoot.nextOrder = localRoot.nextOrder.nextOrder; // next nextOrder reference
    			  }
    		  }
    		  // if localRoot.previousOrder is not null, this node is not the leftmost node 
    		  else {
    			  // create a new link between nodes and jump over delete node
    			  localRoot.previousOrder.nextOrder = localRoot.nextOrder;
    			  localRoot.nextOrder.previousOrder = localRoot.previousOrder;
    		  }
    	  }
    	  
    	  // If there is no left child, return right child
    	  // which can also be null.
    	  return localRoot.right;
      }
      else if (localRoot.right == null) {
    	  // if deleteNode is the leftmost leaf, its previousOrder points null
    	  if(localRoot.previousOrder == null)
    		  localRoot.nextOrder.previousOrder = null;
    	  // if deleteNode is not the leftmost leaf, its previousOrder doesn't point null
    	  else {
    		  // create a new link between nodes and jump over delete node
    		  localRoot.nextOrder.previousOrder = localRoot.previousOrder;
    		  localRoot.previousOrder.nextOrder = localRoot.nextOrder;
    	  }
        
    	  // If there is no right child, return left child.
    	  return localRoot.left;
      }
      else {
    	  // Node being deleted has 2 children, replace the data
    	  // with inorder predecessor.
    	  if (localRoot.left.right == null) {
    		  // The left child has no right child.
    		  // Replace the data with the data in the
    		  // left child.
    		  localRoot.data = localRoot.left.data;
    		 
    		  // check localRoot.left.left, if there is a node
    		  // create a new link between nodes and jump over delete node
    		  if(localRoot.left.left != null) {
    			  localRoot.left.left.nextOrder = localRoot;
    			  localRoot.previousOrder = localRoot.left.left;
    		  }
    		  else {
    			  localRoot.previousOrder.previousOrder.nextOrder = localRoot;
    			  localRoot.previousOrder = localRoot.previousOrder.previousOrder;
    		  }
    		  
    		  // Replace the left child with its left child.
    		  localRoot.left = localRoot.left.left;
          
    		  return localRoot;
    	  }
    	  else {
    		  // Search for the inorder predecessor (ip) and
    		  // replace deleted node’s data with ip.
    		  Node <E> tempNode = findLargestChild(localRoot.left);
    		  // set localRoot data
    		  localRoot.setData(tempNode.data);
    		  // link nextOrder and previousOrder between nodes
    		  tempNode.previousOrder.nextOrder = tempNode.nextOrder;
    		  tempNode.nextOrder.previousOrder = tempNode.previousOrder;
          
    		  return localRoot;
    	  }
      }
    }
  }

  /**** BEGIN EXERCISE ****/
  /** Removes target from tree.
       @param target Item to be removed
       @return true if the object was in the tree, false otherwise
       @post target is not in the tree
       @throws ClassCastException if target is not Comparable
   */
  public boolean remove(E target) {
    return delete(target) != null;
  }

  /** Determine if an item is in the tree
      @param target Item being sought in tree
      @return true If the item is in the tree, false otherwise
      @throws ClassCastException if target is not Comparable
   */
  public boolean contains(E target) {
    return find(target) != null;
  }

  /**** END EXERCISE ****/

  /** Find the node that is the
      inorder predecessor and replace it
      with its left child (if any).
      post: The inorder predecessor is removed from the tree.
      @param parent The parent of possible inorder
                    predecessor (ip)
      @return The data in the ip
   */
  private Node<E> findLargestChild(Node < E > parent) {
    // If the right child has no right child, it is
    // the inorder predecessor.
    if (parent.right.right == null) {
      Node<E> returnValue = parent.right;
      parent.right = parent.right.left;
      return returnValue;
    }
    else {
      return findLargestChild(parent.right);
    }
  }

  /**
   * sort binary search tree
   */
  public void sortBinarySearchTree() {
	  sortBinarySearchTree(root);
	  return;
  }
  
  /**
   * sort binary search tree in O(n) time complexity
   * @param localRoot Root of tree
   */
  private void sortBinarySearchTree(Node <E> localRoot) {
	  // if localRoot is null, print warning message and return
	  if(localRoot == null) {
		  System.out.println("There is not element to sort");
		  return;
	  }
	  // if localRoot.left is not null, go the leftmost node in tree
	  // because this node has the smallest data in tree
	  // sort process starts from there
	  else if(localRoot.left != null) {
		  sortBinarySearchTree(localRoot.left); // call method recursively
	  }
	  // if node is the leftmost node, start to sort binary search tree
	  else {
		  // go to until localRoot.nextOrder is null
		  // print sorted list
		  while(localRoot.nextOrder != null) {
			  System.out.print(localRoot.data + " -> ");
			  localRoot = localRoot.nextOrder;
		  }
		  // print last node
		  System.out.println(localRoot.data);
		  
		  return;
	  }
  }
}
