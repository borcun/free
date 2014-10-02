t, Twitter Client Application
=============================

t is twitter client application which you can manage your account, tweet, display tweets.
it is based-on ruby, so the ruby have to be installed in order to use t application.

1. First of all, you have to install ruby-dev package. After that, t have to be installed via gem.<br>
   sudo apt-get / yum install ruby-dev<br>
   sudo gem install t
   
2. Next, visit https://dev.twitter.com/apps/new page to create a twitter application. There are
   mandatory areas such as name, description and website. In addition, the default access mode is just
   read. To tweet, you have to change application type from read to read and write from 'Permission' tab.

3. Run 't authorize' command which requests consumer key and consumer secret values. These values are in
   'Key and Access Tokens' tab. Enter them successively.
   
4. In this step, the application requests a PIN code, so it opens a new web page for PIN code. Copy and paste
	 it to terminal.
	 
5. After these log in operations, the account is set. 't account' command gives you your account information.<br>
   't whois @USER'      		 : shows USER account information. <br>
   't stream timeline'  		 : lists your timeline. <br>
   't update "Message"  		 : post "Message" tweet <br>
   't follow @USER'     		 : follow USER account <br>
   't mentions'							 : lists current 20 mentions about you <br>
	 't mentions -n 10'   		 : lists last 10 mentions about you <br>
	 't search timeline "word" : search tweets which contains "word" keyword <br>
   't friends USER'			     : lists friends of USER account <br>
	 't favorites USER'				 : return 20 most recent favorite tweet of USER account
