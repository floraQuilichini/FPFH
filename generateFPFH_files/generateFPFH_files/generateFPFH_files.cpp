#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <stdio.h>
#include <string>
#include <set>


std::string getFileName(const std::string& s) {

	char sep = '/';

#ifdef _WIN32
	sep = '\\';
#endif

	size_t i = s.rfind(sep, s.length());
	if (i != std::string::npos) {
		std::string name_ext = s.substr(i + 1, s.length() - i);

		size_t lastindex = name_ext.find_last_of(".");
		return(name_ext.substr(0, lastindex));
	}

	return("");
}


std::string getFileDir(const std::string& s) {

	char sep = '/';

#ifdef _WIN32
	sep = '\\';
#endif

	size_t i = s.rfind(sep, s.length());
	if (i != std::string::npos) {
		std::string name_ext = s.substr(i + 1, s.length() - i);

		std::size_t pos = s.find(name_ext);
		return(s.substr(0, pos));
	}

	return("");
}


struct PointXYZcomp {
	bool operator() (const pcl::PointXYZ& p, const pcl::PointXYZ& q) const
	{
		if (p.x < q.x)
			return true;
		else
		{
			if (p.x == q.x)
			{
				if (p.y < q.y)
					return true;
				else
				{
					if (p.y == q.y)
					{
						if (p.z < q.z)
							return true;
						else
							return false;
					}

					return false;
				}
			}

			return false;
		}

	}
};


// pcd input file must have their coordinates in float type (SIZE 4 4 4). 
// otherwise it won't be read
int main(int argc, const char** argv)
{

	// check the input
	if (argc < 3) {
		PCL_ERROR("you must enter first the filepath of the point cloud where to compute the fpfh"); 
		PCL_ERROR("you must enter secondly the filepath of the point cloud used to approximate the surface");
		PCL_ERROR("there is another optional argument that you can enter; it's the size of voxel for subsampling");
		return (-1);
	}

	// read fpfh pcd file
	std::string pc_filepath = argv[1];
	std::cout << "file path : " << pc_filepath << std::endl;
	std::string file_dir = getFileDir(pc_filepath);
	std::cout << "file dir : " << file_dir << std::endl;
	std::string file_name =  getFileName(pc_filepath);
	std::cout << "file name : " << file_name << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>); // cloud of points

	//if (pcl::io::loadPCDFile<pcl::PointXYZ>(pc_filepath, *object) == -1) //* load the file
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(pc_filepath, *object) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read ply file \n");
		return (-1);
	}

	std::cout << "Loaded "
		<< object->width * object->height
		<< " data points from fpfh_pcd.pcd "
		<< std::endl;

	// read features ply file
	std::string surface_pc_filepath = argv[2];
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_pc(new pcl::PointCloud<pcl::PointXYZ>); // cloud of points

	if (surface_pc_filepath.compare("-") == 0)
		surface_pc = NULL;
	else
	{
		//if (pcl::io::loadPCDFile<pcl::PointXYZ>(surface_pc_filepath, *surface_pc) == -1) //* load the file
		if (pcl::io::loadPLYFile<pcl::PointXYZ>(surface_pc_filepath, *surface_pc) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read ply file \n");
			return (-1);
		}

		std::cout << "Loaded "
			<< surface_pc->width * surface_pc->height
			<< " data points from features_pcd.pcd "
			<< std::endl;
	}


	// subsample the mesh to compute FPFH descriptor only on a reduced number of points
	float voxel_size, r_normals, r_neighboors;
	if (argc == 3){
		voxel_size = 0.01;
		r_normals = 0.02;
		r_neighboors = 0.05;
	}
	else {
		if (argc == 4) {
			voxel_size = std::stof(argv[3]);
			r_normals = 2.0*voxel_size;
			r_neighboors = 5.0*voxel_size;
		}
		else {
			if (argc == 6) {
				voxel_size = std::stof(argv[3]);
				r_normals = std::stof(argv[4]);
				r_neighboors = std::stof(argv[5]);
			}
			else
				std::cerr << "wrong number of arguments " << std::endl;
		}
	}
		

	// subsampling
	pcl::PointCloud<pcl::PointXYZ>::Ptr sub_sample_object(new pcl::PointCloud<pcl::PointXYZ>); // cloud of sub-sampled points
	if (voxel_size == 0.0) {
		if (argc == 4) {
			r_normals = 0.02;
			r_neighboors = 0.05;
		}
		sub_sample_object = object;
	}
	else {
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(object);
		sor.setLeafSize(voxel_size, voxel_size, voxel_size);
		sor.filter(*sub_sample_object);
	}


	// get the number of points where to compute the normals
	pcl::PointCloud<pcl::PointXYZ>::Ptr feature_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (surface_pc)
	{
		std::set<pcl::PointXYZ, PointXYZcomp> unique_points;
		// get all the neighbooring points of the input cloud
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(surface_pc);


		for (size_t i = 0; i < sub_sample_object->size(); ++i)
		{
			std::vector< int > k_neighboors_indices;
			std::vector< float > k_sqr_distances;
			pcl::PointXYZ neighboors_query_point = sub_sample_object->points[i];
			unique_points.insert(neighboors_query_point);
			// find neighbooring points where to compute the normal
			kdtree.radiusSearch(neighboors_query_point, r_neighboors, k_neighboors_indices, k_sqr_distances);
			for (size_t j = 0; j < k_neighboors_indices.size(); ++j)
			{
				pcl::PointXYZ normal_query_point = surface_pc->points[k_neighboors_indices[j]];
				unique_points.insert(normal_query_point);
			}
		}

		for (auto it = unique_points.begin(); it != unique_points.end(); ++it)
				feature_point_cloud->push_back(*it);
	}


	// compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());  //cloud of normals

    // Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	if (surface_pc)
	{
		ne.setInputCloud(feature_point_cloud);
		ne.setSearchSurface(surface_pc);
	}
	else
		ne.setInputCloud(sub_sample_object);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_search_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(normal_search_tree);

	// Use all neighbors in a sphere of radius 2cm by default
	ne.setRadiusSearch(r_normals);

	// Compute the features
	ne.compute(*normals);


	// compute FPFH
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setInputCloud(sub_sample_object);
	if (surface_pc)
		fest.setSearchSurface(feature_point_cloud);

	fest.setInputNormals(normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr neighboors_search_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	fest.setSearchMethod(neighboors_search_tree);

	fest.setRadiusSearch(r_neighboors); // the feature radius can be a parameter

	// output dataset
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());

	// Compute the features
	fest.compute(*object_features);

	// write feature histograms in binary file 
	std::string binary_filename = file_dir;
	FILE* fid = fopen(binary_filename.append(file_name).append(".bin").c_str(), "wb");
	int nV = (int)sub_sample_object->size(), nDim = 33;
	std::cout << "nb subsample points : " << nV << std::endl;
	fwrite(&nV, sizeof(int), 1, fid);
	fwrite(&nDim, sizeof(int), 1, fid);
	for (int v = 0; v < nV; v++) {
		const pcl::PointXYZ &pt = sub_sample_object->points[v];
		float xyz[3] = { pt.x, pt.y, pt.z };
		fwrite(xyz, sizeof(float), 3, fid);
		const pcl::FPFHSignature33 &feature = object_features->points[v];
		fwrite(feature.histogram, sizeof(float), 33, fid);
	}

	fclose(fid);


	// write feature histograms in text file
	std::string text_filename = file_dir;
	std::ofstream text_file;
	text_file.open(text_filename.append(file_name).append("_fpfh.txt").c_str(), std::ios::out);
	text_file << nV << std::endl;
	text_file << nDim << std::endl;
	for (int v = 0; v < nV; v++) {
		const pcl::PointXYZ &pt = sub_sample_object->points[v];
		text_file << pt.x << std::endl;
		text_file << pt.y << std::endl;
		text_file << pt.z << std::endl;
		const pcl::FPFHSignature33 &feature = object_features->points[v];
		for (int i = 0; i < 33; i++)
		{
			text_file << feature.histogram[i] << std::endl;
		}
	}


	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());


	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	fpfh.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch(0.05);

	// Compute the features
	fpfh.compute(*fpfhs);

	// fpfhs->points.size () should have the same size as the input cloud->points.size ()*
	return 0;
	*/

}


/*

// pcd input file must have their coordinates in float type (SIZE 4 4 4). 
// otherwise it won't be read
int main(int argc, const char** argv)
{

	// check the input
	if (argc < 2) {
		PCL_ERROR("you must enter the filepath of the point cloud"); // take as input a pcd file
		PCL_ERROR("there are others optional arguments; first one is original point cloud filename (before downsampling); second one is voxel size; the two other ones are normal and neighboor radii");
		PCL_ERROR("if you don't want to use non downsampled point cloud to compute radii, just enter '_' ");
		return (-1);
	}

	// read pcd file
	std::string pc_filepath = argv[1];
	std::cout << "file path : " << pc_filepath << std::endl;
	std::string file_dir = getFileDir(pc_filepath);
	std::cout << "file dir : " << file_dir << std::endl;
	std::string file_name = getFileName(pc_filepath);
	std::cout << "file name : " << file_name << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr subsampled_object(new pcl::PointCloud<pcl::PointXYZ>); // cloud of points

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pc_filepath, *subsampled_object) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read pcd file \n");
		return (-1);
	}

	std::cout << "Loaded "
		<< subsampled_object->width * subsampled_object->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;


	pcl::PointCloud<pcl::PointXYZ>::Ptr initial_pc(new pcl::PointCloud<pcl::PointXYZ>);
	if (argc > 2) {

		std::string pc_filepath = argv[2];
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(pc_filepath, *initial_pc) == -1) {
			initial_pc = NULL;
		}
	}



	float r_normals, r_neighboors;
	if (argc == 3)
	{
		r_normals = 0.02;
		r_neighboors = 0.05;
	}
	else {
		if (argc == 4) {
			float voxel_size = std::stof(argv[3]);
			r_normals = 2.0*voxel_size;
			r_neighboors = 5.0*voxel_size;
		}
		else {
			if (argc == 5) {
				r_normals = std::stof(argv[3]);
				r_neighboors = std::stof(argv[4]);
			}
			else
				std::cerr << "wrong number of arguments " << std::endl;
		}
	}


	// compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	if (initial_pc)
		ne.setInputCloud(initial_pc);
	else
		ne.setInputCloud(subsampled_object);
	

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Use all neighbors in a sphere of radius 2cm by default
	ne.setRadiusSearch(r_normals);

	// Compute the features
	ne.compute(*normals);


	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
	if (initial_pc)
		fest.setInputCloud(initial_pc);
	else
		fest.setInputCloud(subsampled_object);
	fest.setInputNormals(normals);

	// output dataset
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());

	// Use all neighbors in a sphere of radius 5cm by default
	fest.setRadiusSearch(r_neighboors); // the feature radius can be a parameter

	// Compute the features
	fest.compute(*object_features);

	// write feature histograms in binary file 
	std::string binary_filename = file_dir;
	FILE* fid = fopen(binary_filename.append(file_name).append(".bin").c_str(), "wb");
	int nV = subsampled_object->size(), nDim = 33;
	std::cout << "nb subsample points : " << nV << std::endl;
	fwrite(&nV, sizeof(int), 1, fid);
	fwrite(&nDim, sizeof(int), 1, fid);
	for (int v = 0; v < nV; v++) {
		const pcl::PointXYZ &pt = subsampled_object->points[v];
		float xyz[3] = { pt.x, pt.y, pt.z };
		fwrite(xyz, sizeof(float), 3, fid);
		const pcl::FPFHSignature33 &feature = object_features->points[v];
		fwrite(feature.histogram, sizeof(float), 33, fid);
	}

	fclose(fid);


	// write feature histograms in text file
	std::string text_filename = file_dir;
	std::ofstream text_file;
	text_file.open(text_filename.append(file_name).append("_fpfh.txt").c_str(), std::ios::out);
	text_file << nV << std::endl;
	text_file << nDim << std::endl;
	for (int v = 0; v < nV; v++) {
		const pcl::PointXYZ &pt = subsampled_object->points[v];
		text_file << pt.x << std::endl;
		text_file << pt.y << std::endl;
		text_file << pt.z << std::endl;
		const pcl::FPFHSignature33 &feature = object_features->points[v];
		for (int i = 0; i < 33; i++)
		{
			text_file << feature.histogram[i] << std::endl;
		}
	}

}*/