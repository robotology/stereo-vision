#DispModule.thrift
service DispModule {
	
	bool save_calibration();
	bool calibrate();
	bool toggle_gui();
	bool save_parameters();
	bool default_parameters();
	
	bool set_blf(1:string state);
	bool set_wls(1:string state);
	bool set_alg(1:string alg);

	list<Point3D> rect(1:i32 tlx, 2:i32 tly, 3:i32, w, 4:i32 h);
	list<Point3D> points(1:list<Point3D>);
	list<Point3D> cart2stereo(1:list<Point3D>);
}