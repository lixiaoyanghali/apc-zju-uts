; Auto-generated. Do not edit!


(cl:in-package pr_msgs-srv)


;//! \htmlinclude AskUser-request.msg.html

(cl:defclass <AskUser-request> (roslisp-msg-protocol:ros-message)
  ((question
    :reader question
    :initarg :question
    :type cl:string
    :initform "")
   (choices
    :reader choices
    :initarg :choices
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass AskUser-request (<AskUser-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AskUser-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AskUser-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pr_msgs-srv:<AskUser-request> is deprecated: use pr_msgs-srv:AskUser-request instead.")))

(cl:ensure-generic-function 'question-val :lambda-list '(m))
(cl:defmethod question-val ((m <AskUser-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pr_msgs-srv:question-val is deprecated.  Use pr_msgs-srv:question instead.")
  (question m))

(cl:ensure-generic-function 'choices-val :lambda-list '(m))
(cl:defmethod choices-val ((m <AskUser-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pr_msgs-srv:choices-val is deprecated.  Use pr_msgs-srv:choices instead.")
  (choices m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AskUser-request>) ostream)
  "Serializes a message object of type '<AskUser-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'question))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'question))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'choices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'choices))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AskUser-request>) istream)
  "Deserializes a message object of type '<AskUser-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'question) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'question) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'choices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'choices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AskUser-request>)))
  "Returns string type for a service object of type '<AskUser-request>"
  "pr_msgs/AskUserRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AskUser-request)))
  "Returns string type for a service object of type 'AskUser-request"
  "pr_msgs/AskUserRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AskUser-request>)))
  "Returns md5sum for a message object of type '<AskUser-request>"
  "61358f4df54523fed10b2c62d42335a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AskUser-request)))
  "Returns md5sum for a message object of type 'AskUser-request"
  "61358f4df54523fed10b2c62d42335a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AskUser-request>)))
  "Returns full string definition for message of type '<AskUser-request>"
  (cl:format cl:nil "string question~%string[] choices~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AskUser-request)))
  "Returns full string definition for message of type 'AskUser-request"
  (cl:format cl:nil "string question~%string[] choices~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AskUser-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'question))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'choices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AskUser-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AskUser-request
    (cl:cons ':question (question msg))
    (cl:cons ':choices (choices msg))
))
;//! \htmlinclude AskUser-response.msg.html

(cl:defclass <AskUser-response> (roslisp-msg-protocol:ros-message)
  ((answer
    :reader answer
    :initarg :answer
    :type cl:string
    :initform ""))
)

(cl:defclass AskUser-response (<AskUser-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AskUser-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AskUser-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pr_msgs-srv:<AskUser-response> is deprecated: use pr_msgs-srv:AskUser-response instead.")))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <AskUser-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pr_msgs-srv:answer-val is deprecated.  Use pr_msgs-srv:answer instead.")
  (answer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AskUser-response>) ostream)
  "Serializes a message object of type '<AskUser-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'answer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'answer))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AskUser-response>) istream)
  "Deserializes a message object of type '<AskUser-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'answer) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'answer) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AskUser-response>)))
  "Returns string type for a service object of type '<AskUser-response>"
  "pr_msgs/AskUserResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AskUser-response)))
  "Returns string type for a service object of type 'AskUser-response"
  "pr_msgs/AskUserResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AskUser-response>)))
  "Returns md5sum for a message object of type '<AskUser-response>"
  "61358f4df54523fed10b2c62d42335a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AskUser-response)))
  "Returns md5sum for a message object of type 'AskUser-response"
  "61358f4df54523fed10b2c62d42335a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AskUser-response>)))
  "Returns full string definition for message of type '<AskUser-response>"
  (cl:format cl:nil "string answer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AskUser-response)))
  "Returns full string definition for message of type 'AskUser-response"
  (cl:format cl:nil "string answer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AskUser-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'answer))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AskUser-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AskUser-response
    (cl:cons ':answer (answer msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AskUser)))
  'AskUser-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AskUser)))
  'AskUser-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AskUser)))
  "Returns string type for a service object of type '<AskUser>"
  "pr_msgs/AskUser")