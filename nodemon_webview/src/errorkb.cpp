
/***************************************************************************
 *  errorkb.cpp - Webview extension for nodemon error KB viewing
 *
 *  Created: Tue May 10 11:28:47 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <ros/ros.h>
#include <webview_msgs/NavRegistration.h>
#include <webview_msgs/UrlRegistration.h>
#include <webview_msgs/ProcessRequest.h>

#include <webview/formatters/tracwiki.h>

#include <rospack/rospack.h>

#include <cstdio>
#include <csignal>
#include <regex.h>

#define NODEMON_URL_PREFIX "/errorkb"
#define NODEMON_SERVICE_NAME "/nodemon/errorkb_webview"
#define NODEMON_NAV_ENTRY "Error KB"

class NodemonWebviewErrorKB;
NodemonWebviewErrorKB *g_errorkb;


class NodemonWebviewErrorKB
{
public:
  NodemonWebviewErrorKB()
  {
    // This is a workaround for uninitialized variabls in rospack::ROSPack,
    // cf. https://code.ros.org/trac/ros/ticket/3488
    const char *argv[] = {"", ""};
    try {
      __rospack.run(2, (char **)argv);
    } catch (std::runtime_error &e) {} // ignored

    __srv_proc =
      __n.advertiseService(NODEMON_SERVICE_NAME,
			   &NodemonWebviewErrorKB::process_request_cb, this);

    ros::ServiceClient clt_reg =
      __n.serviceClient<webview_msgs::UrlRegistration>("/webview/register");
    webview_msgs::UrlRegistration srv;
    srv.request.url_prefix = NODEMON_URL_PREFIX;
    srv.request.service_name = NODEMON_SERVICE_NAME;

    if (! clt_reg.call(srv)) {
      throw ros::Exception("Client registration service call failed");
    } else if (! srv.response.success) {
      throw ros::Exception(std::string("Registration failed: ")
			   + srv.response.error);
    }

    /*
    ros::ServiceClient clt_addnav =
      __n.serviceClient<webview_msgs::NavRegistration>("/webview/add_nav_entry");
    webview_msgs::NavRegistration srv_addnav;
    srv_addnav.request.url = NODEMON_URL_PREFIX;
    srv_addnav.request.name = NODEMON_NAV_ENTRY;

    if (! clt_addnav.call(srv_addnav)) {
      printf("Nav entry adding service call failed\n");
    } else if (! srv_addnav.response.success) {
      printf("Adding nav entry failed: %s\n", srv_addnav.response.error.c_str());
    }
    */
  }


  void shutdown()
  {
    ros::ServiceClient clt_ureg =
      __n.serviceClient<webview_msgs::UrlRegistration>("/webview/unregister");
    webview_msgs::UrlRegistration usrv;
    usrv.request.url_prefix = NODEMON_URL_PREFIX;
    usrv.request.service_name = NODEMON_SERVICE_NAME;
  
    if (! clt_ureg.call(usrv)) {
      printf("Client DE-registration service call failed\n");
    } else if (! usrv.response.success) {
      printf("DE-registration failed: %s\n", usrv.response.error.c_str());
    }

    /*
    ros::ServiceClient clt_remnav =
      __n.serviceClient<webview_msgs::NavRegistration>("/webview/remove_nav_entry");
    webview_msgs::NavRegistration srv_remnav;
    srv_remnav.request.url = NODEMON_URL_PREFIX;
    srv_remnav.request.name = NODEMON_NAV_ENTRY;

    if (! clt_remnav.call(srv_remnav)) {
      printf("Nav entry removing service call failed\n");
    } else if (! srv_remnav.response.success) {
      printf("Removing nav entry failed: %s\n", srv_remnav.response.error.c_str());
    }
    */
  }

  bool
  process_request_cb(webview_msgs::ProcessRequest::Request  &req,
		     webview_msgs::ProcessRequest::Response &resp)
  {
    printf("Processing request for %s\n", req.url.c_str());
    // split URL in prefix, package, error ID
    std::string::size_type pos1, pos2;

    if ( ((pos1 = req.url.find("/", 1)) == std::string::npos) ||
	 ((pos2 = req.url.find("/", pos1 + 1)) == std::string::npos) )
    {
      resp.code = webview_msgs::ProcessRequest::Response::HTTP_BAD_REQUEST;
      resp.error = "Malformed URL";
      return true;
    }

    std::string package = req.url.substr(pos1 + 1, pos2 - pos1 - 1);
    std::string errorid = req.url.substr(pos2 + 1);

    // Check package name and error ID as a very basic security layer
    regex_t re;
    int regerr = 0;
    if ( (regerr = regcomp(&re, "^[a-zA-Z0-9_-]+$", REG_EXTENDED)) != 0 ) {
      char errtmp[1024];
      regerror(regerr, &re, errtmp, sizeof(errtmp));
      resp.code = webview_msgs::ProcessRequest::Response::HTTP_INTERNAL_SERVER_ERROR;
      resp.error = std::string("Failed to compile regex: ") + errtmp;
      return true;
    }

    if (regexec(&re, package.c_str(), 0, NULL, 0) == REG_NOMATCH) {
      resp.code = webview_msgs::ProcessRequest::Response::HTTP_BAD_REQUEST;
      resp.error = "Invalid package name";
      return true;
    }

    if (regexec(&re, errorid.c_str(), 0, NULL, 0) == REG_NOMATCH) {
      resp.code = webview_msgs::ProcessRequest::Response::HTTP_BAD_REQUEST;
      resp.error = "Invalid error ID";
      return true;
    }
    regfree(&re);

    rospack::Package *pkg = NULL;
    try {
      pkg = __rospack.get_pkg(package);
    } catch (ros::Exception &e) {
      resp.code = webview_msgs::ProcessRequest::Response::HTTP_INTERNAL_SERVER_ERROR;
      resp.error = e.what();
      return true;
    } catch (std::runtime_error &e) {
      resp.code = webview_msgs::ProcessRequest::Response::HTTP_INTERNAL_SERVER_ERROR;
      resp.error = e.what();
      return true;
    }

    if (pkg) {
      std::string path = pkg->flags("error_kb", "path");
      std::string::size_type endpos = path.find_last_not_of(' ');
      path = path.substr(0, endpos+1);
      
      if (path == "") {
	resp.code = webview_msgs::ProcessRequest::Response::HTTP_NOT_FOUND;
	resp.error = std::string("Package ") + package
	  + " does not provide an error knowledge base";
	return true;
      }

      // Find specific documentation
      std::string docpath = pkg->path + "/" + path + "/doc/" + errorid + ".txt";
      FILE *f = fopen(docpath.c_str(), "r");
      if (!f) {
	resp.code = webview_msgs::ProcessRequest::Response::HTTP_NOT_FOUND;
	resp.error = std::string("No documentation for error ")
	  + package + "/" + errorid;
	return true;
      }

      std::string body;
      while (! feof(f) && ! ferror(f)) {
	char tmp[1024];
	size_t nr = fread(tmp, 1, sizeof(tmp), f);
	if (nr > 0) {
	  body.append(tmp, nr);
	}
      }
      fclose(f);

      // Find generic information
      std::string gendoc = "";
      std::string genpath = pkg->path + "/" + path + "/doc/generic.txt";
      f = fopen(genpath.c_str(), "r");
      if (f) {
	while (! feof(f) && ! ferror(f)) {
	  char tmp[1024];
	  size_t nr = fread(tmp, 1, sizeof(tmp), f);
	  if (nr > 0) {
	    gendoc.append(tmp, nr);
	  }
	}
	fclose(f);
      }

      std::string formatted_body = __twf.format(body);

      if (gendoc != "") {
	    resp.html_header =
	      "  <link type=\"text/css\" href=\"/static/css/jqtheme/jquery-ui.custom.css\" rel=\"stylesheet\" />\n"
	      "  <script type=\"text/javascript\" src=\"/static/js/jquery.min.js\"></script>\n"
	      "  <script type=\"text/javascript\" src=\"/static/js/jquery-ui.custom.min.js\"></script>\n";


	gendoc =
	  "<script type=\"text/javascript\">\n"
	  "  $(function(){\n"
	  "    $(\"#gendoc-title\").click(function(){\n"
	  "	     if ( $(\"#gendoc\").is(\":visible\") ) {\n"
	  "        $(\"#gendoc\").hide(\"blind\");\n"
	  "        $(\"#gendoc-icon\").attr(\"src\", \"/static/images/icon-triangle-e.png\");\n"
	  "      } else {\n"
	  "	       $(\"#gendoc\").show(\"blind\");\n"
	  "        $(\"#gendoc-icon\").attr(\"src\", \"/static/images/icon-triangle-s.png\");\n"
	  "      }\n"
	  "    });\n"
	  "    $(\"#gendoc\").hide();\n"
	  "  });\n"
	  "</script>\n"
	  "<div id=\"gendoc-box\">\n"
	  "  <div><a id=\"gendoc-title\" href=\"#\"><img id=\"gendoc-icon\" "
	  "class=\"gendoc-icon\" src=\"/static/images/icon-triangle-e.png\" />"
	  "Generic information</a></div>\n"
	  "  <div id=\"gendoc\">\n"
	  + __twf.format(gendoc) + "</div></div>\n";
      }

      resp.code = webview_msgs::ProcessRequest::Response::HTTP_OK;
      resp.wrap_in_page = true;
      resp.title = std::string("Documentation for ")
	+ package + "/" + errorid;
      resp.body = formatted_body + "\n\n" + gendoc;
    } else {
      resp.code = webview_msgs::ProcessRequest::Response::HTTP_NOT_FOUND;
      resp.error = std::string("Package ") + package + " not found";
    }

    return true;
  }

 private:
  ros::NodeHandle  __n;
  rospack::ROSPack  __rospack;

  ros::ServiceServer __srv_proc;

  fawkes::TracWikiHeadingFormatter __twf;
};


void
handle_signal(int signum)
{
  g_errorkb->shutdown();

  ros::shutdown();
}


/** Fawkes application.
 * @param argc argument count
 * @param argv array of arguments
 */
int
main(int argc, char **argv)
{
  ros::init(argc, argv, "nodemon_webview_errorkb", ros::init_options::NoSigintHandler);

  // Note that we have a custom SIGINT handler to allow us to properly
  // unregister from webview! We cannot simply call this when ros::spin()
  // returns because by that time service calls won't work anymore
  signal(SIGINT, handle_signal);

  try {
    g_errorkb = new NodemonWebviewErrorKB();
  } catch (ros::Exception &e) {
    printf("Failed to instantiate: %s\n", e.what());
    return 1;
  }

  ros::spin();
  delete g_errorkb;

  return 0;
}
