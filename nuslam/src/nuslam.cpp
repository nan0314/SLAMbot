#include "nuslam/nuslam.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <cmath>


namespace nuslam{

    std::vector<std::vector<geometry_msgs::Point>> findClusters(std::vector<float> ranges,double max_range, double min_range){

        using rigid2d::deg2rad;
        using rigid2d::rad2deg;
        std::vector<std::vector<geometry_msgs::Point>> clusters;

        int degree = 0;
        double thresh = 0.07;

        while (degree < ranges.size()){

            // Ignore if not within laser range
            if (ranges[degree] > max_range | ranges[degree] < min_range){
                degree+=1;
                continue;
            }

            // If the distance between current point and previous point is less than threshold value
            // add it to the current cluster

            int angle = degree;
            int next = (degree + 1) % 360;

            geometry_msgs::Point point;
            point.x = ranges[angle]*cos(deg2rad(angle));
            point.y = ranges[angle]*sin(deg2rad(angle));
            std::vector<geometry_msgs::Point> cluster = {point};

            double current_dist = sqrt(pow(point.x,2) + pow(point.y,2));
            double next_dist = sqrt(pow(ranges[next]*cos(deg2rad(next)),2) + pow(ranges[next]*sin(deg2rad(next)),2));

            while (fabs(current_dist - next_dist) < thresh){

                angle = next;
                next = (angle + 1) % 360;

                point.x = ranges[angle]*cos(deg2rad(angle));
                point.y = ranges[angle]*sin(deg2rad(angle));

                current_dist = sqrt(pow(point.x,2) + pow(point.y,2));
                next_dist = sqrt(pow(ranges[next]*cos(deg2rad(next)),2) + pow(ranges[next]*sin(deg2rad(next)),2));

                cluster.push_back(point);
            }

            // If the cluster is too small skip it otherwise keep track of that cluster.
            if (cluster.size() > 3 & degree < angle){
                clusters.push_back(cluster);
            } else if (cluster.size() > 3 & degree >= angle){
                std::vector<geometry_msgs::Point> combined;
                combined.reserve(clusters[0].size() + cluster.size());
                combined.insert(combined.end(), clusters[0].begin(),clusters[0].end());
                combined.insert(combined.end(),cluster.begin(),cluster.end());

                clusters[0] = combined;
            }

            // Update degree
            if (degree < angle){
                degree = angle;
            } else if (degree == angle){
                degree += 1;
            } else {
                degree+=angle;
            }

        }

        return clusters;

    }

    bool classifyCircle(std::vector<geometry_msgs::Point> cluster){

        using rigid2d::rad2deg;

        geometry_msgs::Point p2 = cluster[0];
        geometry_msgs::Point p3 = cluster[cluster.size()-1];

        // Find the angles between the arc endpoints and each internal point in degrees
        std::vector<double> angles;
        for (int i = 1; i<cluster.size()-1; i++){

            geometry_msgs::Point p1 = cluster[i];
            double numerator = p2.y*(p1.x-p3.x) + p1.y*(p3.x-p2.x) + p3.y*(p2.x-p1.x);
            double denominator = (p2.x-p1.x)*(p1.x-p3.x) + (p2.y-p1.y)*(p1.y-p3.y);

            double angle = rad2deg(atan(numerator/denominator));

            angles.push_back(angle);
        }

        // Find mean of angles
        double mean = 0;
        for (auto angle : angles){
            mean += angle/angles.size();
        }

        // Calculate std deviation
        double std_dev = 0;
        for (auto angle : angles){
            std_dev += pow(angle - mean,2);
        }

        std_dev = sqrt(std_dev/angles.size());


        if (std_dev < 10){
            return true;
        } else {
            return false;
        }
    }


    visualization_msgs::Marker fitCircle(std::vector<geometry_msgs::Point> circle){

        int id = 0;
        double x_mean = 0;
        double y_mean = 0;
        for (auto point : circle){
            x_mean += point.x/circle.size();
            y_mean += point.y/circle.size();
        }

        // Shift coordinates by the mean
        for (int i = 0; i<circle.size(); i++){
            circle[i].x = circle[i].x - x_mean;
            circle[i].y = circle[i].y - y_mean;
        }

        // Create Z matrixy_mean += cirlce.y/circles.size()
        arma::mat Z(circle.size(),4,arma::fill::ones);
        double z_mean = 0;
        for (int i = 0; i<circle.size(); i++){
            geometry_msgs::Point p = circle[i];
            double z = pow(p.x,2) + pow(p.y,2);
            z_mean += z/circle.size();

            Z(i,0) = z;
            Z(i,1) = p.x;
            Z(i,2) = p.y;
        }

        // Calculate H inverse Matrix
        arma::mat Hinv(4,4,arma::fill::eye);
        Hinv(3,0) = 0.5;
        Hinv(0,3) = 0.5;
        Hinv(0,0) = 0.0;
        Hinv(3,3) = -2*z_mean;

        // Perform SVD
        arma::mat U;
        arma::vec s;
        arma::mat V;

        svd(U,s,V,Z);  

        // Calculate A Matrix
        arma::vec A;
        if (s(3) < 1e-12){
            A = V.col(3);
        } else {
            // arma::mat sig = Z.fill(0);
            // std::cout << Z.size() << std::endl;
            // std::cout << s.size() << std::endl << std::endl << std::endl << std::endl;
            // sig(arma::span(0,s.size()-1),arma::span(0,s.size()-1)) = arma::diagmat(s);
            arma::mat Y = V*arma::diagmat(s)*V.t();
            arma::mat Q = Y*Hinv*Y;

            arma::vec eigval;
            arma::mat eigvec;

            arma::eig_sym(eigval, eigvec, Q);

            int index = 0;
            double val = INT_MAX;
            for (int i = 0; i<eigval.size();i++){
                if (eigval[i]<val & eigval[i] > 0){
                    val = eigval[i];
                    index = i;
                }
            }

            arma::vec Astar = eigvec.col(index);
            A = arma::solve(Y,Astar);
        }

        // Calculate equation for circle
        double a = -A(1)/(2*A(0));
        double b = -A(2)/(2*A(0));
        double R_squared = (pow(A(1),2) + pow(A(2),2) -4*A(0)*A(3)) / (4*pow(A(0),2));
        double tube_radius = sqrt(R_squared);


        // Set up the marker msg for the tube;

        visualization_msgs::Marker tube;
        tube.ns = "real";
        tube.id = id++;
        tube.type = 3;
        tube.action = 0;
        tube.pose.position.x = a + x_mean;
        tube.pose.position.y = b + y_mean;
        tube.scale.x = 2*tube_radius;
        tube.scale.y = 2*tube_radius;
        tube.scale.z = 1;
        tube.color.r = 80./255.;
        tube.color.g = 220./255.;
        tube.color.b = 100./255.;    
        tube.color.a = 1;

        return tube;
    }


    std::vector<double> cartesian2polar(std::vector<double> cartesian){

        std::vector<double> polar;

        polar.push_back(pow(pow(cartesian[0],2) + pow(cartesian[1],2),0.5));
        polar.push_back(atan2(cartesian[1],cartesian[0]));

        return polar;
    }


    void Filter::initialize_uncertainty(){
        uncertainty = arma::mat(2*n + 3, 2*n + 3, arma::fill::eye);
        uncertainty*=INT_MAX;
        uncertainty(0,0) = 0;
        uncertainty(1,1) = 0;
        uncertainty(2,2) = 0;
    }

    Filter::Filter() : n(10) {
        initialize_uncertainty();
        estimated_xi  = arma::vec(3+2*n,arma::fill::zeros);
        estimated_xi(0) = 0;
        estimated_xi(1) = 0;
        estimated_xi(2) = 0;
        Q = arma::mat(2*n+3, 2*n + 3, arma::fill::zeros);
    }

    Filter::Filter(double n, arma::mat Q_in, arma::mat R) : n(n), R(R) {
        initialize_uncertainty();
        estimated_xi  = arma::vec(3+2*n,arma::fill::ones);
        estimated_xi(0) = 0;
        estimated_xi(1) = 0;
        estimated_xi(2) = 0;
        this->Q = arma::mat(2*n+3, 2*n + 3, arma::fill::zeros);
        for (int i = 0; i<Q_in.n_rows; i++){
            for (int j = 0; j<Q_in.n_cols; j++){
                this->Q(i,j) = Q_in(i,j);
            }
        }
        
    }

    arma::mat Filter::getUncertainty(){
        return uncertainty;
    }

    arma::mat Filter::getQ(){
        return Q;
    }

    arma::mat Filter::getR(){
        return R;
    }

    arma::vec Filter::getEstimate(){
        return estimated_xi;
    }


    arma::vec Filter::predict(const rigid2d::Twist2D& u_t){

        // Calculate the A matrix
        // arma::mat A_t = A(u_t);
        arma::mat A_t = arma::mat(3+2*n,3+2*n,arma::fill::eye);
        double th = estimated_xi(0);

        if (fabs(u_t.dth) < .000001){
            A_t(1,0) += -u_t.dx*sin(th);
            A_t(2,0) += u_t.dx*cos(th);

            estimated_xi(0) = estimated_xi(0) + 0;
            estimated_xi(1) = estimated_xi(1) + u_t.dx * cos(th);
            estimated_xi(2) = estimated_xi(2) + u_t.dx * sin(th);

        } else {
            A_t(1,0) += -u_t.dx/u_t.dth*cos(th) + u_t.dx/u_t.dth*cos(th + u_t.dth);
            A_t(2,0) += -u_t.dx/u_t.dth*sin(th) + u_t.dx/u_t.dth*sin(th + u_t.dth);

            estimated_xi(0) = estimated_xi(0) + u_t.dth;
            estimated_xi(1) = estimated_xi(1) + -u_t.dx/u_t.dth*sin(th) + u_t.dx/u_t.dth*sin(th + u_t.dth);
            estimated_xi(2) = estimated_xi(2) + u_t.dx/u_t.dth*cos(th) - u_t.dx/u_t.dth*cos(th + u_t.dth);
        }

        // Set the estimated_xi to the prediction
        estimated_xi(0) = rigid2d::normalize_angle(estimated_xi(0));

        // Propogate the uncertainty
        uncertainty = A_t*uncertainty*A_t.t() + Q;

        return estimated_xi;
    }

    void Filter::initialize_landmark(arma::vec z_i,int j){
        estimated_xi(3+2*j) = estimated_xi(1) + z_i(0)*cos(z_i(1) + estimated_xi(0));
        estimated_xi(4+2*j) = estimated_xi(2) + z_i(0)*sin(z_i(1) + estimated_xi(0));

        return;
    }

    arma::mat Filter::A(const rigid2d::Twist2D& u_t){

        arma::mat out(2*n+3,2*n+3,arma::fill::eye);
        double th_tm = rigid2d::normalize_angle(estimated_xi(0));

        if (rigid2d::almost_equal(u_t.dth,0)){
            out(1,0) += -u_t.dx*sin(th_tm);
            out(2,0) += u_t.dx*cos(th_tm);
        } else {
            out(1,0) += -u_t.dx/u_t.dth*cos(th_tm) + u_t.dx/u_t.dth*cos(th_tm+u_t.dth);
            out(2,0) += -u_t.dx/u_t.dth*sin(th_tm) + u_t.dx/u_t.dth*sin(th_tm+u_t.dth);
        }

        return out;
    }

    arma::mat Filter::H(int j){

        double delx = estimated_xi(3+2*j) - estimated_xi(1);
        double dely = estimated_xi(4+2*j) - estimated_xi(2);
        double d = pow(delx,2)+pow(dely,2);

        arma::mat out(2,3+2*n,arma::fill::zeros);

        out(0,1) = -delx/pow(d,0.5);
        out(0,2) = -dely/pow(d,0.5);
        out(0,3+2*j) = delx/pow(d,0.5);
        out(0,4+2*j) = dely/pow(d,0.5);

        out(1,0) = -1;
        out(1,1) = dely/d;
        out(1,2) = -delx/d;
        out(1,3+2*j) = -dely/d;
        out(1,4+2*j) = delx/d;

        return out;
    }


    arma::vec Filter::h(int j){

        arma::vec z(2);
        std::vector<double> cartesian;
        std::vector<double> polar;

        cartesian.push_back(estimated_xi(3+2*j) - estimated_xi(1));
        cartesian.push_back(estimated_xi(4+2*j) - estimated_xi(2));

        // Calculate z_i
        polar = nuslam::cartesian2polar(cartesian);

        z(0) = polar[0];
        z(1) = rigid2d::normalize_angle(polar[1] - estimated_xi(0));
        
        return z;
    }


    arma::vec Filter::update(rigid2d::Twist2D u_t, arma::vec z_i, int j){

        // Compute estimate measurement for jth landmark
        arma::vec z_est = h(j);

        // Compute Kalman gain
        arma::mat H_i = H(j);
        arma::mat K_i = uncertainty*arma::trans(H_i) * arma::inv(H_i*uncertainty*arma::trans(H_i) + R);

        // Refine the estimated state
        arma::vec dz = z_i - z_est;
        dz[1] = rigid2d::normalize_angle(dz[1]);
        estimated_xi = estimated_xi + K_i*dz;
        estimated_xi(0) = rigid2d::normalize_angle(estimated_xi(0));

        // Update the uncertainty matrix
        uncertainty = (arma::mat(3+2*n, 3+2*n,arma::fill::eye) - K_i*H_i) * uncertainty;

        return estimated_xi;
    }


}