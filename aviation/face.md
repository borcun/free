## Future Airborne Capability Environment

 - FACE was formed in 2010 to define an open avionics environment for all military
   airborne platform types.
 
 - Government-industry software standard and business strategy for acquisition
   of affordable software systems that promote innovation and rapid integration of
   portable capabilities across programs.
 
 - The use of standard interfaces that will lead to reuse of capabilities.

 - Portability of applications across multiple FACE systems and vendors.

 - Making safety-critical computing operations more robust, interoperable, portable,
   and secure.
   
 - The latest edition promotes application interoperability and portability with
   enhanced requirements for exchanging data among FACE compontents, including a 
   formally specified data model and emphasis on defining common language requirements
   for the standard. This architecture defines standardized interfaces to allow
   software compontents to be moved between systems, including those developed by
   different vendors. The standardized interfaces follow a data architecture to ensure
   the data communicated between the software components is fully described to facilitate
   their integration on new systems.
   
 - The FACE Reference Architecture is composed of logical segments where variance occurs.
   The structure created by connecting these sgments together is the foundation of FACE
   Reference Architecture. The five segment of the FACE Reference Architecture are;

    1. Operating System Segment (OSS)
	2. I/O Services Segment (IOSS)
	3. Platform-Specific Services Segment (PSSS)
	4. Transport Service Segment (TSS)
	5. Portable Components Segment (PCS)
  
   FACE Reference Architecture defines a set of standardized interfaces providing connections 
   between the FACE architectural segments.
   
 - The FACE Reference Architecture defines three FACE OSS Profiles tailoring the Operating System (OS) 
   Application Programming Interfaces (APIs), programming languages, programming language features, 
   run-times, frameworks, and graphics capabilities to meet the requirements of software components 
   for differing levels of criticality. The three Profiles are Security, Safety, and General Purpose. 
   The Security Profile constrains the OS APIs to a minimal useful set allowing assessment for 
   high-assurance security functions executing as a single process. The Safety Profile is less 
   restrictive than the Security Profile and constrains the OS APIs to those that have a safety 
   certification pedigree. The General Purpose Profile is the least constrained profile and supports 
   OS APIs meeting real-time deterministic or non-real-time, non-deterministic requirements depending 
   on the system or subsystem implementation. 

 - The FACE Data Architecture defines the FACE Data Model Language (including the language binding 
   specification), Query and Template language, FACE Shared Data Model (SDM) and the rules of 
   construction of the Unit of Portability (UoP) Supplied Model (USM). Each PCS Unit of Conformance 
   (UoC), PSSS UoC, or TSS UoC providing using TS Interfaces is accompanied by a USM consistent with 
   the FACE SDM and defines its interfaces in terms of the FACE Data Model Language. A Domain-Specific 
   Data Model (DSDM) captures content relevant to a domain of interest and can be used as a basis for USMs.
