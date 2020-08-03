//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef MARKER_DETECTOR_HPP
#define MARKER_DETECTOR_HPP

#include <opencv2/core/core.hpp>
#include <cstdio>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;
using namespace Eigen;

class  Marker: public vector<Point2f>
{
public:
    //id of  the marker
    int id;
    //size of the markers sides in meters
    float metric_size;
    //matrices of rotation and translation respect to the camera
    cv::Mat Rvec,Tvec;

    Isometry3f pose;

    Marker();
    Marker(const Marker &M);
    Marker(const vector<Point2f> &corners,int _id=-1);
    ~Marker() {}

    bool isValid()const{return id!=-1 && size()==4;}
    void draw(cv::Mat &in, cv::Scalar color, int lineWidth=1,bool writeId=true)const;

    void calculateExtrinsics(float markerSize,cv::Mat  CameraMatrix,cv::Mat Distorsion)throw(cv::Exception);

    cv::Point2f getCenter()const;
    float getPerimeter()const;
    float getArea()const;

    friend bool operator<(const Marker &M1,const Marker&M2){return M1.id<M2.id;}

    friend ostream & operator<<(ostream &str,const Marker &M)
    {
        str<<M.id<<"=";
        for (int i=0;i<4;i++)
            str<<"("<<M[i].x<< ","<<M[i].y<<") ";
        str<<"Txyz=";
        for (int i=0;i<3;i++)
            str<<M.Tvec.ptr<float>(0)[i]<<" ";
        str<<"Rxyz=";
        for (int i=0;i<3;i++)
            str<<M.Rvec.ptr<float>(0)[i]<<" ";

        return str;
    }

};

struct Marker3D:public vector<cv::Point3f> {
    Marker3D() {}
    Marker3D(int _id) {id=_id; }
    Marker3D(const Marker3D&MI): vector<Point3f>(MI){id=MI.id; }
    Marker3D & operator=(const Marker3D&MI){vector<Point3f> ::operator=(MI);id=MI.id;return *this;}
    int id;
};

class  Board
{
public:

    Board();
    Board(string filePath)throw (cv::Exception);
    Board(const Board  &T);
    Board & operator=(const Board  &T);
    void saveToFile(string sfile);
    void readFromFile(string sfile);
    int getIndexOfMarkerId(int id);
    const Marker3D& getMarkerInfo(int id);
    void getIdList(vector<int> &ids,bool append=true)const;

    vector<Marker3D> reference;
    vector<Marker> detected;
    cv::Mat Rvec,Tvec;
    int w,h;
    Isometry3f pose;
    Vector3f plane_point, plane_normal;

};

class SubPixelCorner
{
private:
    int _winSize,_apertureSize,_max_iters;
    cv::TermCriteria _term;
    double eps;
    cv::Mat mask;
public:
    bool enable;
    SubPixelCorner();
    void checkTerm();
    double pointDist(cv::Point2f estimate_corner,cv::Point2f curr_corner);
    void RefineCorner(cv::Mat image,std::vector <cv::Point2f> &corners);
    void generateMask();
};

class MarkerCode {
public:

    MarkerCode(uint n=0);
    MarkerCode(const MarkerCode &MC);

    // Get id of a specific rotation as the number obtaiend from the concatenation of all the bits
    uint getId(uint rot=0) { return _ids[rot]; }

    // Get a bit value in a specific rotation. Refered as a unidimensional string of bits, i.e. pos=y*n+x
    bool get(uint pos, uint rot=0) { return _bits[rot][pos]; }

    //Get the string of bits for a specific rotation
    std::vector<bool> getRotation(uint rot) { return _bits[rot]; }

    void set(unsigned int pos, bool val);
    uint size() {return n()*n(); }
    uint n() {return _n; }
    uint selfDistance(unsigned int &minRot);
    uint selfDistance() {uint minRot;return selfDistance(minRot);}
    uint distance(MarkerCode m, unsigned int &minRot);
    uint distance(MarkerCode m) {uint minRot;return distance(m, minRot);}
    void fromString(std::string s);
    std::string toString();
    cv::Mat getImg(unsigned int pixSize);

private:
    unsigned int _ids[4]; // ids in the four rotations
    vector<bool> _bits[4]; // bit strings in the four rotations
    unsigned int _n; // marker dimension

    uint hammingDistance(vector<bool> m1, vector<bool> m2);

};


class Dictionary : public std::vector<MarkerCode> {
public:

    bool fromFile(std::string filename);
    bool toFile(std::string filename);
    void dumpMarkers();

    uint distance(MarkerCode m, unsigned int &minMarker, unsigned int &minRot);
    uint distance(MarkerCode m) {uint minMarker, minRot;return distance(m,minMarker,minRot);}
    uint minimumDistance();

private:

    // convert to string
    template <class T> static std::string toStr(T num) {
        std::stringstream ss;
        ss << num;
        return ss.str();
    }
};



// Balanced Binary Tree for a marker dictionary
class BalancedBinaryTree {

public:

    void loadDictionary(Dictionary &D);
    bool findId(unsigned int id, unsigned int &orgPos);

private:

    vector< pair<uint,uint> > _orderD; // dictionary sorted by id, first element is the id, second element is the position in original D
    vector< pair<int, int> > _binaryTree; // binary tree itself (as a vector), each element is a node of the tree
    // first element indicate the position in _binaryTree of the lower child
    // second element is the position in _binaryTree of the higher child
    // -1 value indicates no lower or higher child
    int _root; // position in _binaryTree of the root node of the tree

};





/**\brief Main class for marker detection
 *
 */
class  MarkerDetector
{
    //Represent a candidate to be a maker
    class MarkerCandidate : public Marker{
    public:
        MarkerCandidate(){}
        MarkerCandidate(const Marker &M): Marker(M){}
        MarkerCandidate(const  MarkerCandidate &M): Marker(M){
            contour=M.contour;
            idx=M.idx;
        }
        MarkerCandidate & operator=(const  MarkerCandidate &M){
            (*(Marker*)this)=(*(Marker*)&M);
            contour=M.contour;
            idx=M.idx;
            return *this;
        }

        vector<cv::Point> contour;//all the points of its contour
        int idx;//index position in the global contour list
    };
public:

    MarkerDetector();
    MarkerDetector(Mat camera, Mat distortion);
    ~MarkerDetector();

	int getNumDetected() {return num_detected;}

    void init(Matrix3f camera);

    float detect(const Mat &input,float markerSizeMeters) throw (cv::Exception);

    bool createDictionary(string filename, uint dictSize, uint n);
    bool createBoard(string boardfile, int w, int h);

    bool loadDictionary(string filename);
    bool loadBoard(string filename);


    enum ThresholdMethods {FIXED_THRES,ADPT_THRES,CANNY};


    void setThresholdMethod(ThresholdMethods m) {_thresMethod=m;}
    ThresholdMethods getThresholdMethod()const {return _thresMethod;}

    /**
     * Set the parameters of the threshold method
     * We are currently using the Adptive threshold
     *   @param param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
     *   @param param2: The constant subtracted from the mean or weighted mean
     */
    void setThresholdParams(double param1,double param2) {_thresParam1=param1;_thresParam2=param2;}



    const cv::Mat & getThresholdedImage() {return thres;}

    enum CornerRefinementMethod {NONE,HARRIS,SUBPIX,LINES};

    void setCornerRefinementMethod(CornerRefinementMethod method) {_cornerMethod=method;}

    CornerRefinementMethod getCornerRefinementMethod()const {return _cornerMethod;}


    /**Specifies the min and max sizes of the markers as a fraction of the image size. By size we mean the maximum
     * of cols and rows.
     * @param min size of the contour to consider a possible marker as valid (0,1]
     * @param max size of the contour to consider a possible marker as valid [0,1)
     *
     */
    void setMinMaxSize(float mini=0.03,float maxi=0.5){_minSize=max(0.0f,mini);_maxSize=min(0.0f,maxi);}
    void getMinMaxSize(float &min,float &max){min=_minSize;max=_maxSize;}
    
    // Enables/Disables erosion process that is REQUIRED for chessboard like boards.
    void enableErosion(bool enable){_doErosion=enable;}


    void setDesiredSpeed(int val);
    int getDesiredSpeed()const {return _speed;}
    
    // Specifies size for the canonical marker image. Bigger is slower. Minimum 10, Default 56
    void setWarpSize(int val) {_markerWarpSize = max(10,val);}
    int getWarpSize()const {return _markerWarpSize;}

    int identifyMarker(const cv::Mat &in,int &nRotations);
    void pyrDown(unsigned int level){pyrdown_level=level;}


    //Returns a list candidates to be markers (rectangles), for which no valid id was found after calling detectRectangles
    const vector<vector<Point2f> > &getCandidates() {return _candidates;}

    void draw3dAxis(cv::Mat &Image,Marker &m);
    void draw3dCube(cv::Mat &Image,Marker &m);
    void drawBoardAxis(cv::Mat &Image);
    void drawBoardCube(cv::Mat &Image);

    void dumpBoardImage(string filename, uint pixelpermarker);
    

    cv::Mat camera, distortion;
    vector<Marker> detectedMarkers;
    Dictionary D;
    Board board;

    void set_repj_err_thres(float Repj_err_thres){_repj_err_thres=Repj_err_thres;}
    float get_repj_err_thres  ( )const {return _repj_err_thres;}


private:

	float num_detected; // Mira 21.Oct.

    //Given the input image with markers, creates an output image with it in the canonical position
    bool warp(cv::Mat &in,cv::Mat &out,cv::Size size, vector<Point2f> points)throw (cv::Exception);

    // Refine MarkerCandidate Corner using LINES method
    void refineCandidateLines(MarkerCandidate &candidate);


    //Thesholds the passed image with the specified method.
    void thresHold(int method,const cv::Mat &grey,cv::Mat &thresImg,double param1=-1,double param2=-1);

    // This function returns in candidates all the rectangles found in a thresolded image
    void detectRectangles(const cv::Mat &thresImg,vector<vector<Point2f> > & candidates);


    void detectMarkers(const Mat &input,float markerSizeMeters) throw (cv::Exception);
    void detectRectangles(const cv::Mat &thresImg,vector<MarkerCandidate> & candidates);
    //Current threshold method
    ThresholdMethods _thresMethod;
    //Threshold parameters
    double _thresParam1,_thresParam2;
    //Current corner method
    CornerRefinementMethod _cornerMethod;
    //minimum and maximum size of a contour lenght
    float _minSize,_maxSize;

    BalancedBinaryTree _binaryTree;
    unsigned int _n,_ncellsBorder,_correctionDistance;
    int _swidth; // cell size in the canonical image

    float _repj_err_thres;

    int _speed;
    int _markerWarpSize;
    bool _doErosion;
    float _borderDistThres; //border around image limits in which corners are not allowed to be detected.
    vector<std::vector<cv::Point2f> > _candidates;     // Rectangles that have no valid id
    int pyrdown_level;

    cv::Mat grey,thres,thres2,reduced;

    bool isInto(cv::Mat &contour,std::vector<cv::Point2f> &b);
    int perimeter(std::vector<cv::Point2f> &a);

    // auxiliar functions to perform LINES refinement
    void interpolate2Dline( const vector< cv::Point2f > &inPoints, cv::Point3f &outLine);
    cv::Point2f getCrossPoint(const cv::Point3f& line1, const cv::Point3f& line2);
    void distortPoints(vector<cv::Point2f> in, vector<cv::Point2f> &out, const cv::Mat &camMatrix, const cv::Mat &distCoeff);
    
    void draw(cv::Mat out,const std::vector<Marker> &markers );


    //int threads = 1;
    //int curr_thread = 0;
	int threads;
	int curr_thread;
    template<typename T> void joinVectors(vector<vector<T> >  &vv,vector<T> &v,bool clearv=false){
        if (clearv) v.clear();
        for(size_t i=0;i<vv.size();i++)
            for(size_t j=0;j<vv[i].size();j++) v.push_back(vv[i][j]);
    }
};

#endif