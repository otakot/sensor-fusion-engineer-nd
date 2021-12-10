/// @brief KD Tree implementation

#pragma once

/// @brief Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT point, int setId)
	:	point(point), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	std::pair<float, float> getCompareCoordinatesForLevel(size_t level, const PointT& target_point, const PointT& tree_root_point) const
	{
		auto split_dimension = level % 3;
		return std::make_pair(target_point.data[split_dimension], tree_root_point.data[split_dimension]);
	}

	void addPointAsTreeNode(Node<PointT>*& tree_root, std::size_t level, PointT point, int id)
	{
      	if(tree_root == NULL)
      	{
        	tree_root = new Node<PointT>(point,id);
      	}
      	else
	  	{
			// assert point.size == root.point.size 
			float point_coord, tree_root_point_coord;
			std::tie(point_coord, tree_root_point_coord) = getCompareCoordinatesForLevel(level, point, tree_root->point);

			auto& new_tree_root = (point_coord < tree_root_point_coord) ? tree_root->left : tree_root->right;
			addPointAsTreeNode(new_tree_root, level + 1, point, id);
	  	}
   	}

	void insert(PointT point, int id)
	{
		addPointAsTreeNode(this->root, 0 /* root level */, point, id); 
	}

	bool isPointInsideBoundaryBox(const PointT& point, const PointT& box_center_point, float box_boundary_dist_from_center) const
	{
		return point.x > box_center_point.x - box_boundary_dist_from_center &&
				point.x < box_center_point.x + box_boundary_dist_from_center &&
				point.y > box_center_point.y - box_boundary_dist_from_center &&
				point.y < box_center_point.y + box_boundary_dist_from_center &&
				point.z > box_center_point.z - box_boundary_dist_from_center &&
				point.z < box_center_point.z + box_boundary_dist_from_center;
	}

	float calculateDistanceBetweenPoints(const PointT& point_1, const PointT& point_2) const
	{
		return sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) + (point_1.y - point_2.y) * (point_1.y - point_2.y) + (point_1.z - point_2.z) * (point_1.z - point_2.z));
	}

	void findNearbyPointsInTree(Node<PointT>* const& tree_root, std::size_t tree_level, const PointT& target_point, float distanceTol, std::vector<int>& result) const
	{
		if(tree_root != NULL)
		{
			// if current tree point is within tolerance distance to target point add its id to result;
			if(isPointInsideBoundaryBox(tree_root->point, target_point, distanceTol))
			{
				auto distance_to_point = calculateDistanceBetweenPoints(tree_root->point, target_point);
				if (distance_to_point < distanceTol)
				{
					result.push_back(tree_root->id);
				}
			}

			// now continue search of nearby tree points in each of split areas of current tree point which overlap theboundary box of target point
			float target_point_coord, tree_root_point_coord;
			std::tie(target_point_coord, tree_root_point_coord) = getCompareCoordinatesForLevel(tree_level, target_point, tree_root->point);

			// check if current tree point's coordinate splitter crosses left/lower part of target point's boundary box
			if (tree_root_point_coord > target_point_coord - distanceTol)
			{
				findNearbyPointsInTree(tree_root->left, tree_level + 1, target_point, distanceTol, result);
			}
			// check if current tree point's coordinate splitter crosses right/upper part of target point's boundary box
			if (tree_root_point_coord < target_point_coord + distanceTol)
			{
				findNearbyPointsInTree(tree_root->right, tree_level + 1, target_point, distanceTol, result);
			}
		}
	}

	/// @brief return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const PointT& target, float distanceTol) const
	{
		std::vector<int> nearby_point_ids{};
		findNearbyPointsInTree(this->root, 0, target, distanceTol, nearby_point_ids);
		return nearby_point_ids;
	}
};