#ifndef SELECT_BIN_HPP
#define SELECT_BIN_HPP

#include <vector>
#include <iostream>
#include <cstdio>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/geometry/polygon_operations.h>

#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace pcl;
using namespace Eigen;
using namespace std;
template<typename PointT>
class PinPicking{
    typename PointCloud<PointT>::Ptr cloudPtr;
    typename PointCloud<PointT>::Ptr planePtr;
    typename PointCloud<PointT>::Ptr objectPtr;

    visualization::PCLVisualizer::Ptr cloudViewer;
    visualization::ImageViewer::Ptr imageViewer;

    vector<PointT> supportPts;  // points for support plane
    vector<PointT> framePts;    // points for the frame of the bin
    vector<int> supportPtsIndices;
    vector<int> framePtsIndices;

    typename search::Search<PointT>::Ptr searchPtr;

    typename EdgeAwarePlaneComparator<PointT, Normal>::Ptr planeComparator;
    PointIndices::Ptr planeIndices;
    unsigned char* rgb_data_;
    std::vector<float> distanceMap;

    bool ppClick;   // 0 for support plane click
                    // 1 for frame click
    int framePtsCount, supportPtsCount;
    bool selected;
public:
    PinPicking( typename PointCloud<PointT>::Ptr cloudPtr_ )
        : cloudPtr(cloudPtr_)
        , planeComparator(new EdgeAwarePlaneComparator<PointT, Normal>){
        planeComparator->setDistanceThreshold (0.01f, false);
        framePtsCount = 0;
        supportPtsCount = 0;
        selected = false;
    }

    /** \brief estimated normals given a point cloud
      * \param[in] point cloud
      * \param[out] normal point cloud
      */
    void estimateNormals(const typename PointCloud<PointT>::ConstPtr &input, PointCloud<Normal> &normals) {
        if (input->isOrganized ()) {
            IntegralImageNormalEstimation<PointT, Normal> ne;
            ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
            ne.setMaxDepthChangeFactor (0.02f);
            ne.setNormalSmoothingSize (20.0f);
            ne.setInputCloud (input);
            ne.compute (normals);
            // Save the distance map for the plane comparator
            float *map=ne.getDistanceMap ();// This will be deallocated with the IntegralImageNormalEstimation object...
            distanceMap.assign(map, map+input->size() ); //...so we must copy the data out
            planeComparator->setDistanceMap(distanceMap.data());
        }
        else {
            NormalEstimation<PointT, Normal> ne;
            ne.setInputCloud (input);
            ne.setRadiusSearch (0.02f);
            ne.setSearchMethod (searchPtr);
            ne.compute (normals);
        }
    }


    /** \brief Given a plane, and the set of inlier indices representing it,
      * segment out the object of intererest supported by it.
      *
      * \param[in] picked_idx the index of a point on the object
      * \param[in] cloud the full point cloud dataset
      * \param[in] plane_indices a set of indices representing the plane supporting the object of interest
      * \param[out] object the segmented resultant object
      */
    void segmentObject (int picked_idx,  const typename PointCloud<PointT>::ConstPtr &cloud,
                        const PointIndices::Ptr &plane_indices,  PointCloud<PointT> &object) {
        typename PointCloud<PointT>::Ptr plane_hull (new PointCloud<PointT>);

        // Compute the convex hull of the plane
        ConvexHull<PointT> chull;
        chull.setDimension (2);
        chull.setInputCloud (cloudPtr);
        chull.setIndices (planeIndices);
        chull.reconstruct (*plane_hull);

        // Remove the plane indices from the data
        typename PointCloud<PointT>::Ptr plane (new PointCloud<PointT>);
        ExtractIndices<PointT> extract (true);
        extract.setInputCloud (cloudPtr);
        extract.setIndices (planeIndices);
        extract.setNegative (false);
        extract.filter (*plane);
        PointIndices::Ptr indices_but_the_plane (new PointIndices);
        extract.getRemovedIndices (*indices_but_the_plane);

        // Extract all clusters above the hull
        PointIndices::Ptr points_above_plane (new PointIndices);
        ExtractPolygonalPrismData<PointT> exppd;
        exppd.setInputCloud (cloudPtr);
        exppd.setIndices (indices_but_the_plane);
        exppd.setInputPlanarHull (plane_hull);
        exppd.setViewPoint (cloud->points[picked_idx].x, cloud->points[picked_idx].y, cloud->points[picked_idx].z);
        exppd.setHeightLimits (0.001, 0.5);           // up to half a meter
        exppd.segment (*points_above_plane);

        vector<PointIndices> euclidean_label_indices;
        // Prefer a faster method if the cloud is organized, over EuclidanClusterExtraction
        if (cloudPtr->isOrganized ()) {
            // Use an organized clustering segmentation to extract the individual clusters
            typename EuclideanClusterComparator<PointT, Normal, Label>::Ptr euclidean_cluster_comparator (new EuclideanClusterComparator<PointT, Normal, Label>);
            euclidean_cluster_comparator->setInputCloud (cloud);
            euclidean_cluster_comparator->setDistanceThreshold (0.03f, false);
            // Set the entire scene to false, and the inliers of the objects located on top of the plane to true
            Label l; l.label = 0;
            PointCloud<Label>::Ptr scene (new PointCloud<Label> (cloud->width, cloud->height, l));
            // Mask the objects that we want to split into clusters
            for (int i = 0; i < static_cast<int> (points_above_plane->indices.size ()); ++i)
                scene->points[points_above_plane->indices[i]].label = 1;
            euclidean_cluster_comparator->setLabels (scene);

            vector<bool> exclude_labels (2);  exclude_labels[0] = true; exclude_labels[1] = false;
            euclidean_cluster_comparator->setExcludeLabels (exclude_labels);

            OrganizedConnectedComponentSegmentation<PointT, Label> euclidean_segmentation (euclidean_cluster_comparator);
            euclidean_segmentation.setInputCloud (cloud);

            PointCloud<Label> euclidean_labels;
            euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
        }
        else
        {
            console::print_highlight (stderr, "Extracting individual clusters from the points above the reference plane ");
            console::TicToc tt; tt.tic ();

            EuclideanClusterExtraction<PointT> ec;
            ec.setClusterTolerance (0.02); // 2cm
            ec.setMinClusterSize (100);
            ec.setSearchMethod (searchPtr);
            ec.setInputCloud (cloudPtr);
            ec.setIndices (points_above_plane);
            ec.extract (euclidean_label_indices);

            console::print_info ("[done, ");
            console::print_value ("%g", tt.toc ());
            console::print_info (" ms : ");
            console::print_value ("%lu", euclidean_label_indices.size ());
            console::print_info (" clusters]\n");
        }

        // For each cluster found
        bool cluster_found = false;
        for (size_t i = 0; i < euclidean_label_indices.size (); i++) {
            if (cluster_found)
                break;
            // Check if the point that we picked belongs to it
            for (size_t j = 0; j < euclidean_label_indices[i].indices.size (); ++j) {
                if (picked_idx != euclidean_label_indices[i].indices[j])
                    continue;
                copyPointCloud (*cloud, euclidean_label_indices[i].indices, object);
                cluster_found = true;
                break;
            }
        }
    }


    /** \brief segment point cloud given a selected point
      * \param[in] picked point
      * \param[in] picked point index
      * \param[out] extracted planar region
      * \param[out] segmented object on the plane
      */
    void segment (const PointT &picked_point, int picked_idx,
                  PlanarRegion<PointT> &region,
                  typename PointCloud<PointT>::Ptr &object) {
        object.reset ();
        vector<ModelCoefficients> modelCoefficients;
        vector<PointIndices> inlierIndices, boundaryIndices;
        vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT> > > regions;
        // Prefer a faster method if the cloud is organized, over RANSAC
        if (cloudPtr->isOrganized ()) {
            console::print_highlight (stderr, "Estimating normals ");
            console::TicToc tt; tt.tic ();
            // Estimate normals
            PointCloud<Normal>::Ptr normalCloud (new PointCloud<Normal>);
            estimateNormals (cloudPtr, *normalCloud);

            console::print_info ("[done, ");
            console::print_value ("%g", tt.toc ());
            console::print_info (" ms : ");
            console::print_value ("%lu", normalCloud->size());
            console::print_info (" points]\n");

            OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps;
            mps.setMinInliers (1000);
            mps.setAngularThreshold (deg2rad (3.0)); // 3 degrees
            mps.setDistanceThreshold (0.03); // 2 cm
            mps.setMaximumCurvature (0.001); // a small curvature
            mps.setProjectPoints (true);
            mps.setComparator (planeComparator);
            mps.setInputNormals (normalCloud);
            mps.setInputCloud (cloudPtr);

            // Use one of the overloaded segmentAndRefine calls to get all the information that we want out
            PointCloud<Label>::Ptr labels (new PointCloud<Label>);
            vector<PointIndices> labelIndices;
            mps.segmentAndRefine (regions, modelCoefficients, inlierIndices, labels, labelIndices, boundaryIndices);
        }
        else {
            SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (SACMODEL_PLANE);
            seg.setMethodType (SAC_RANSAC);
            seg.setMaxIterations (10000);
            seg.setDistanceThreshold (0.005);

            // Copy XYZ and Normals to a new cloud
            typename PointCloud<PointT>::Ptr cloudSegmented (new PointCloud<PointT> (*cloudPtr));
            typename PointCloud<PointT>::Ptr cloudRemaining (new PointCloud<PointT>);

            ModelCoefficients coefficients;
            ExtractIndices<PointT> extract;
            PointIndices::Ptr inliers (new PointIndices ());

            // Up until 30% of the original cloud is left
            int i = 1;
            while (double (cloudSegmented->size ()) > 0.3 * double (cloudPtr->size ())) {
                seg.setInputCloud (cloudSegmented);

                console::print_highlight (stderr, "Searching for the largest plane (%2.0d) ", i++);
                console::TicToc tt; tt.tic ();
                seg.segment (*inliers, coefficients);

                console::print_info ("[done, ");
                console::print_value ("%g", tt.toc ());
                console::print_info (" ms : ");
                console::print_value ("%lu", inliers->indices.size ());
                console::print_info (" points]\n");

                // No datasets could be found anymore
                if (inliers->indices.empty ())
                    break;

                // Save this plane
                PlanarRegion<PointT> region;
                region.setCoefficients (coefficients);
                regions.push_back (region);

                inlierIndices.push_back (*inliers);
                modelCoefficients.push_back (coefficients);

                // Extract the outliers
                extract.setInputCloud (cloudSegmented);
                extract.setIndices (inliers);
                extract.setNegative (true);
                extract.filter (*cloudRemaining);
                cloudSegmented.swap (cloudRemaining);
            }
        }
        console::print_highlight ("Number of planar regions detected: %lu for a cloud of %lu points\n", regions.size (), cloudPtr->size ());
        double max_dist = numeric_limits<double>::max ();
        // Compute the distances from all the planar regions to the picked point, and select the closest region
        int idx = -1;
        for (size_t i = 0; i < regions.size (); ++i) {
            double dist = pointToPlaneDistance (picked_point, regions[i].getCoefficients ());
            if (dist < max_dist) {
                max_dist = dist;
                idx = static_cast<int> (i);
            }
        }

        // Get the plane that holds the object of interest
        if (idx != -1) {
            planeIndices.reset (new PointIndices (inlierIndices[idx]));

            if (cloudPtr->isOrganized ()) {
                approximatePolygon (regions[idx], region, 0.01f, false, true);
                console::print_highlight ("Planar region: %lu points initial, %lu points after refinement.\n", regions[idx].getContour ().size (), region.getContour ().size ());
            }
            else {
                // Save the current region
                region = regions[idx];
                console::print_highlight (stderr, "Obtaining the boundary points for the region ");
                console::TicToc tt; tt.tic ();
                // Project the inliers to obtain a better hull
                typename PointCloud<PointT>::Ptr cloudProjected (new PointCloud<PointT>);
                ModelCoefficients::Ptr coefficients (new ModelCoefficients (modelCoefficients[idx]));
                ProjectInliers<PointT> proj;
                proj.setModelType (SACMODEL_PLANE);
                proj.setInputCloud (cloudPtr);
                proj.setIndices (planeIndices);
                proj.setModelCoefficients (coefficients);
                proj.filter (*cloudProjected);

                // Compute the boundary points as a ConvexHull
                ConvexHull<PointT> chull;
                chull.setDimension (2);
                chull.setInputCloud (cloudProjected);
                PointCloud<PointT> planeHull;
                chull.reconstruct (planeHull);
                region.setContour (planeHull);

                console::print_info ("[done, ");
                console::print_value ("%g", tt.toc ());
                console::print_info (" ms : ");
                console::print_value ("%lu", planeHull.size ());
                console::print_info (" points]\n");
            }
        }
    }


    /** \brief calculate the point inside the frame
      * \param[in] x direction vector
      * \param[in] y direction vector
      * \param[in] z direction vector
      * \param[in] center pt of the frame
      * \param[out] indices for the inlier pts
      */
    vector<int> getInFrame( Vector3f framex, Vector3f framey, Vector3f framez, Vector3f center) {
        Vector3f ptVec;
        float framexlen = framex.norm();
        float frameylen = framey.norm();
        vector<int> indices;
        for ( size_t i = 0; i < cloudPtr->points.size(); ++ i ) {
            if ( !pcl_isfinite(cloudPtr->points[i].x) &&
                 !pcl_isfinite(cloudPtr->points[i].y) &&
                 !pcl_isfinite(cloudPtr->points[i].z) ) {
                ptVec = cloudPtr->points[i].getVector3fMap() - center;
                Vector3f projectedOnZ = ((ptVec.dot(framez))/(framez.norm()*framez.norm()))*framez;
                Vector3f projectedOnPlane = ptVec - projectedOnZ;
                Vector3f projectedOnX = ((projectedOnPlane.dot(framex))/(framexlen*framexlen))*framex;
                Vector3f projectedOnY = ((projectedOnPlane.dot(framey))/(frameylen*frameylen))*framey;
                if ( projectedOnX.norm() < frameylen/2 && projectedOnY.norm() < frameylen ) {
                    console::print_info("%d ", i);
                    indices.push_back( i );
                }
            }
        }
        console::print_info("\n");
        return indices;
    }

    /** \brief point picking callback function: get called when
      * user click a point with shift.
      * \param[in] event that triggered the call
      */
    void pointPickCallback( const visualization::PointPickingEvent & event, void * ) {
        int idx = event.getPointIndex();
        if ( idx == -1 )
            return;
        if ( selected == false ) {
            if ( ppClick == false ) {
                vector<int> indices(1);
                vector<float> distances(1);
                // Get the point that was picked
                PointXYZRGB pickedPt;
                event.getPoint( pickedPt.x, pickedPt.y, pickedPt.z );
                stringstream ss;
                ss << "sphere_" << idx;
                cloudViewer->addSphere(pickedPt, 0.002, 1.0, 0.0, 0.0, ss.str());
                searchPtr->nearestKSearch (pickedPt, 1, indices, distances);
                if (imageViewer) {
                    uint32_t width  = searchPtr->getInputCloud ()->width;
                    int v = indices[0] / width, u = indices[0] % width;
                    imageViewer->addCircle (u, v, 1, 1.0, 0.0, 0.0, "circles", 1.0);
                    imageViewer->markPoint (u, v, visualization::red_color, visualization::blue_color);
                }
                supportPts.push_back( pickedPt );
                supportPtsCount ++;
                console::print_info(stderr, "Picked %d support plane point with index %d, and coordinates %f, %f, %f.\n", supportPtsCount, idx, pickedPt.x, pickedPt.y, pickedPt.z);

                if ( supportPtsCount == 3 )
                    ppClick = true;
            }
            else if (ppClick == true) {
                vector<int> indices(1);
                vector<float> distances(1);
                // Get the point that was picked
                PointT pickedPt;
                event.getPoint( pickedPt.x, pickedPt.y, pickedPt.z );
                stringstream ss;
                ss << "sphere_" << idx;
                cloudViewer->addSphere(pickedPt, 0.002, 1.0, 0.0, 0.0, ss.str());
                searchPtr->nearestKSearch (pickedPt, 1, indices, distances);
                if (imageViewer) {
                    uint32_t width  = searchPtr->getInputCloud ()->width;
                    int v = indices[0] / width, u = indices[0] % width;
                    imageViewer->addCircle (u, v, 1, 1.0, 0.0, 0.0, "circles", 1.0);
                    imageViewer->markPoint (u, v, visualization::red_color, visualization::blue_color);
                }
                framePts.push_back( pickedPt );
                framePtsCount ++;
                console::print_info(stderr, "Picked %d frame point with index %d, and coordinates %f, %f, %f.\n", framePtsCount, idx, pickedPt.x, pickedPt.y, pickedPt.z);

                if ( framePtsCount == 3 ) {
                    selected = true;
                }
            }
        }
    }


    /** \brief display mask image according to indices
      * \param[in] indices, positive
      */
    void displayMaskImage( vector<int> indices ) {
        cv::Mat maskImage( cloudPtr->height, cloudPtr->width, CV_8UC1, cv::Scalar::all(0) );
        for ( size_t i = 0; i < indices.size(); ++ i ) {
            maskImage.at<uchar>( indices[i]/cloudPtr->width, indices[i]%cloudPtr->width ) = 255;
        }
        cv::namedWindow( "mask image" );
        cv::imshow( "mask image", maskImage );
        cv::waitKey(0);
        maskImage.release();
        cv::destroyWindow( "mask image" );
    }

    /** \brief display point cloud using different clusters
      * \param[in] cluster indices
      * \param[in] point cloud
      */
    void displayClusteredPointCloud( vector<PointIndices> indices, typename PointCloud<PointT>::Ptr filteredCloud ) {
        cout << "clusters: " << indices.size() << endl;
        for ( size_t i = 0; i < indices.size(); ++ i ) {
            uint8_t r = rand()%255;
            uint8_t g = rand()%255;
            uint8_t b = rand()%255;
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            for ( size_t j = 0; j < indices[i].indices.size(); ++ j ) {
                filteredCloud->points[indices[i].indices[j]].rgb = *reinterpret_cast<float*>(&rgb);
            }
        }
        visualization::PCLVisualizer::Ptr viewer( new visualization::PCLVisualizer("cluster") );
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(filteredCloud);
        viewer->addPointCloud<PointT> (filteredCloud, rgb, "cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        while (!viewer->wasStopped ()) {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

    /** \brief Euclidean Cluster Extraction
      * \param[in] input point cloud
      * \param[out] vector of cluster indices
      */
    vector< PointIndices > euclideanCluster( typename PointCloud<PointT>::Ptr filteredCloud ) {
        console::TicToc tt; tt.tic();
        typename search::KdTree<PointT>::Ptr tree( new search::KdTree<PointT> );
        tree->setInputCloud( filteredCloud );
        vector<PointIndices> clusterIndices;
        EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.01); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (filteredCloud);
        ec.extract (clusterIndices);

        PointCloud<PointXYZRGBL>::Ptr labeledCloud( new PointCloud<PointXYZRGBL> () );
        copyPointCloud( *filteredCloud, *labeledCloud );
        for ( size_t i = 0; i < clusterIndices.size(); ++ i ) {
            for ( size_t j = 0; j < clusterIndices[i].indices.size(); ++ j ) {
                labeledCloud->points[clusterIndices[i].indices[j]].label = (int)i;
            }
        }
        io::savePCDFile( "labeled_cloud.pcd", *labeledCloud );
        return clusterIndices;
    }

    /** \brief point picking on image viewer
      * \param[in] event that triggered the call
      */
    void imagePointPickingCallback( const visualization::MouseEvent & event, void * ) {
        if (event.getType() == visualization::MouseEvent::MouseButtonPress && event.getButton() == visualization::MouseEvent::LeftButton) {
            int idx = (cloudPtr->height - event.getY())*cloudPtr->width + event.getX();
            // check whether the 6 points are defined
            if ( selected == false ) {
                // check isfinite
                if ( pcl_isfinite( cloudPtr->points[idx].x ) &&
                     pcl_isfinite( cloudPtr->points[idx].y ) &&
                     pcl_isfinite( cloudPtr->points[idx].z )) {
                    stringstream ss;
                    ss << "sphere_" << idx;
                    if ( ppClick == false ) {
                        cloudViewer->addSphere( cloudPtr->points[idx], 0.01, 1.0, 0.0, 0.0, ss.str() );
                        imageViewer->addCircle( event.getX(), cloudPtr->height - event.getY(), 1, 1.0, 0.0, 0.0, "circles", 1.0 );
                        imageViewer->markPoint( event.getX(), cloudPtr->height - event.getY(), visualization::red_color, visualization::blue_color );
                        supportPtsCount ++;
                        console::print_info(stderr, "Picked %d support plane point with index %d, and coordinates %f, %f, %f.\n", supportPtsCount, idx, cloudPtr->points[idx].x, cloudPtr->points[idx].y, cloudPtr->points[idx].z);
                        supportPtsIndices.push_back( idx );
                        supportPts.push_back( cloudPtr->points[idx] );
                        if ( supportPtsCount == 3 )
                            ppClick = true;
                    }
                    else {
                        cloudViewer->addSphere( cloudPtr->points[idx], 0.01, 0.0, 1.0, 0.0, ss.str() );
                        imageViewer->addCircle( event.getX(), cloudPtr->height-event.getY(), 1, 1.0, 0.0, 0.0, "circles", 1.0 );
                        imageViewer->markPoint( event.getX(), cloudPtr->height-event.getY(), visualization::blue_color, visualization::red_color );
                        framePtsCount ++;
                        console::print_info(stderr, "Picked %d frame point with index %d, and coordinates %f, %f, %f.\n", framePtsCount, idx, cloudPtr->points[idx].x, cloudPtr->points[idx].y, cloudPtr->points[idx].z);
                        framePtsIndices.push_back( idx );
                        framePts.push_back( cloudPtr->points[idx] );
                        if ( framePtsCount == 4 )
                            selected = true;
                    }
                }
                else {
                    console::print_error( "please reclick the point on image\n" );
                }
            }
            else {
                console::print_error( "you can only click less than 7 times\n" );
            }
        }
    }

    /** \brief keyboard callback function
      * \param[in] event that triggered the call
      */
    void keyboardCallback (const visualization::KeyboardEvent& event, void*) {
        if (event.getKeySym() == "c" && event.keyDown()) {
            console::print_info( "Selected points ... ... Done, calculating and seperating\n" );
            // generate opencv image for convex contour
            cv::Mat rgbImage( cloudPtr->height, cloudPtr->width, CV_8UC3 );
            for ( int y = 0; y < cloudPtr->height; ++ y ) {
                for ( int x = 0; x < cloudPtr->width; ++ x ) {
                    PointT & pt = cloudPtr->points[y*cloudPtr->width+x];
                    rgbImage.at<cv::Vec3b>(y,x)[0] = pt.b;
                    rgbImage.at<cv::Vec3b>(y,x)[1] = pt.g;
                    rgbImage.at<cv::Vec3b>(y,x)[2] = pt.r;
                }
            }

            // generate polygon using selected frame points
            cv::Point polygonCorners[1][4];
            polygonCorners[0][0] = cv::Point( framePtsIndices[0]%cloudPtr->width, framePtsIndices[0]/cloudPtr->width );
            polygonCorners[0][1] = cv::Point( framePtsIndices[1]%cloudPtr->width, framePtsIndices[1]/cloudPtr->width );
            polygonCorners[0][2] = cv::Point( framePtsIndices[3]%cloudPtr->width, framePtsIndices[3]/cloudPtr->width );
            polygonCorners[0][3] = cv::Point( framePtsIndices[2]%cloudPtr->width, framePtsIndices[2]/cloudPtr->width );

            const cv::Point* ppt[1] = { polygonCorners[0] };
            int npt[] = {4};
            cv::Mat maskImage( cloudPtr->height, cloudPtr->width, CV_8UC1, cv::Scalar::all(0) );
            cv::fillPoly( maskImage, ppt, npt, 1, cv::Scalar::all(255), 8);


            // generate point indices
            vector<int> inFrameIndices;
            for ( int y = 0; y < maskImage.rows; ++ y )
                for ( int x = 0; x < maskImage.cols; ++ x )
                    if ( maskImage.at<uchar>(y, x) == 255 )
                        if ( pcl_isfinite( cloudPtr->points[y*cloudPtr->width+x].x ) &&
                             pcl_isfinite( cloudPtr->points[y*cloudPtr->width+x].y ) &&
                             pcl_isfinite( cloudPtr->points[y*cloudPtr->width+x].z ))
                            inFrameIndices.push_back( y*maskImage.cols+x );
            // select point above the support plane
            Vector3f supportCtPt = supportPts[0].getVector3fMap();
            Vector3f supportx = supportPts[1].getVector3fMap() - supportPts[0].getVector3fMap();
            Vector3f supporty = supportPts[2].getVector3fMap() - supportPts[0].getVector3fMap();
            Vector3f supportz = supportx.cross(supporty);
            if ( supportz[1] > 0.0 )
                supportz *= -1.0;
            vector<int> abovePlaneIndices;
            for ( size_t i = 0; i < inFrameIndices.size(); ++ i ) {
                PointT & pt = cloudPtr->points[inFrameIndices[i]];
                Vector3f projectedOnZ = (((pt.getVector3fMap()-supportCtPt).dot(supportz))/(supportz.norm()*supportz.norm()))*supportz;
                if ( projectedOnZ.norm() > 0.005 )
                    abovePlaneIndices.push_back( inFrameIndices[i] );
            }

            // select plane further than the frame
            /*
            Vector3f framex   = framePts[1].getVector3fMap() - framePts[0].getVector3fMap();    // width
            Vector3f framey   = framePts[2].getVector3fMap() - framePts[0].getVector3fMap();    // height
            Vector3f framez   = framex.cross( framey );
            Vector3f frameCtPt = framePts[0].getVector3fMap() + framex/2 + framey/2;
            vector<int> furtherFrameIndices;
            for ( size_t i = 0; i < abovePlaneIndices.size(); ++ i ) {
                PointT & pt = cloudPtr->points[abovePlaneIndices[i]];
                Vector3f projectedOnZ = (((pt.getVector3fMap()-frameCtPt).dot(framez))/(framez.norm()*framez.norm()))*framez;
                if ( projectedOnZ.norm() > 0.005 )
                    furtherFrameIndices.push_back( abovePlaneIndices[i] );
            }
            displayMaskImage( furtherFrameIndices );
            */

            typename PointCloud<PointT>::Ptr filteredCloud( new PointCloud<PointT> );
            if ( false ) {
                filteredCloud->width = cloudPtr->width;
                filteredCloud->height= cloudPtr->height;
                filteredCloud->is_dense = cloudPtr->is_dense;
                filteredCloud->points.resize( cloudPtr->points.size() );
                for ( size_t i = 0; i < abovePlaneIndices.size(); ++ i ) {
                    filteredCloud->points[abovePlaneIndices[i]].x = cloudPtr->points[abovePlaneIndices[i]].x;
                    filteredCloud->points[abovePlaneIndices[i]].y = cloudPtr->points[abovePlaneIndices[i]].y;
                    filteredCloud->points[abovePlaneIndices[i]].z = cloudPtr->points[abovePlaneIndices[i]].z;
                    filteredCloud->points[abovePlaneIndices[i]].rgb = cloudPtr->points[abovePlaneIndices[i]].rgb;
                }
            }
            else {
                PointIndices::Ptr inliers (new PointIndices);
                inliers->indices = abovePlaneIndices;
                ExtractIndices<PointT> extract;
                extract.setInputCloud (cloudPtr);
                extract.setIndices (inliers);
                extract.setNegative (false);
                // Get the points associated with the planar surface
                extract.filter (*filteredCloud);
            }

            io::savePCDFile( "filtered_cloud.pcd", *filteredCloud );
            console::print_info("write point cloud to disk\n");
            vector<PointIndices> clusters = euclideanCluster( filteredCloud );
            console::print_info("segment filtered cloud using euclidean clustering method\n");
            displayClusteredPointCloud( clusters, filteredCloud );
            console::print_info("display segmented filtered cloud\n");
        }
    }


    /** \brief cloud viewer and image viewer initialisation
      */
    void initGUI() {
        cloudViewer.reset( new visualization::PCLVisualizer("Point Cloud") );
        if (cloudPtr->isOrganized ()) {
            // If the dataset is organized, and has RGB data, create an image viewer
            vector<pcl::PCLPointField> fields;
            int rgba_index = -1;
            rgba_index = getFieldIndex (*cloudPtr, "rgb", fields);
            if (rgba_index >= 0) {
                imageViewer.reset( new visualization::ImageViewer("Image") );
                imageViewer->registerMouseCallback( &PinPicking::imagePointPickingCallback, *this );
                imageViewer->registerKeyboardCallback( &PinPicking::keyboardCallback, *this );
                imageViewer->setPosition (cloudPtr->width, 0);
                imageViewer->setSize (cloudPtr->width, cloudPtr->height);
                imageViewer->addRGBImage<PointT> (cloudPtr);
            }
            cloudViewer->setSize (cloudPtr->width, cloudPtr->height);
        }
        cloudViewer->registerKeyboardCallback(&PinPicking::keyboardCallback, *this);
        cloudViewer->registerPointPickingCallback (&PinPicking::pointPickCallback, *this);
        cloudViewer->setPosition (0, 0);

        cloudViewer->addPointCloud (cloudPtr, "scene");
        cloudViewer->resetCameraViewpoint ("scene");
        cloudViewer->addCoordinateSystem (0.1, 0, 0, 0);

        searchPtr.reset (new search::OrganizedNeighbor<PointT>);
        searchPtr->setInputCloud (cloudPtr);

    }

    /**  \brief main loop for visualisation
      */
    void compute () {
        while (!cloudViewer->wasStopped ()) {
            cloudViewer->spinOnce ();
            if (imageViewer) {
                imageViewer->spinOnce ();
                if (imageViewer->wasStopped ())
                    break;
            }
            boost::this_thread::sleep (boost::posix_time::microseconds (100));
        }
    }
};

#endif // SELECT_BIN_HPP
