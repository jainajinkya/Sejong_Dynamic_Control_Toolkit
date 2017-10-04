#ifndef SR_RECORDER
#define SR_RECORDER

class CvVideoWriter;

class srRecorder{
 public:
  static srRecorder* GetRecorder();
  ~srRecorder();

  void CaptureCurrentScreen(int fps);


 private:
  CvVideoWriter * writer_;
  int img_width_, img_height_;

  void _InitiateSetup(int fps);
  bool is_first_call_;
  srRecorder();
};

#endif
